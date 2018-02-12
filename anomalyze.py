#!/usr/bin/python
#import matplotlib.pyplot as plt
import sys
import rosbag
import math
import re
import time
import argparse
import matplotlib.pyplot as plt

# ANSI terminal color codes

BOLD = "\x1B[1m"
RED = "\x1B[1;31m"
GRN = "\x1B[1;32m"
YEL = "\x1B[1;33m"
BLU = "\x1B[1;34m"
MAG = "\x1B[1;35m"
CYN = "\x1B[1;36m"
WHT = "\x1B[37m"
RST = "\x1B[0m"

# Tests go here
class AnomalyTest(object):

  def __init__(self):
    pass

  def get_subscribed_channel(self):
    return []
  
  def feed_message(self, topic, msg, t_msg):
    pass

  def found_anomaly(self, desc, start_rostime, end_rostime, msg):

    duration_ms = (end_rostime - start_rostime) / 1e6
    start = time.ctime(start_rostime.to_sec())
    end = time.ctime(end_rostime.to_sec())

    print((YEL+'{}'+RST+' - duration: '+GRN+'{}'+RST+' ms \tstart: '+
          BLU+'{}'+RST+' end: '+BLU+'{}'+RST).format(
      desc, duration_ms, start, end
    ))

  def print_summary(self):
    print('[ default print_summary ]')

  def do_graph(self):
    pass

class ExtendedOutageTest(AnomalyTest):

  MAX_OUTAGE_DURATION = 0 # seconds

  # For each data channel, keep track of outage states...
  in_outage = {}
  outage_start_t = {}
  outage_start_ts = {}
  outage_durations = {}

  def __init__(self):
    super(AnomalyTest,self).__init__()

    for channel in self.get_subscribed_channel():
      self.in_outage[channel] = False
      self.outage_start_t[channel] = False
      self.outage_start_ts[channel] = []
      self.outage_durations[channel] = []
  
  def get_subscribed_channel(self):
    return ['SingleBaselineRTK', 'Attitude2D']
  
  def feed_message(self, topic, msg, t_msg):
    
    # Does the current message have valid data?
    if (msg.bitfield & 1<<0) == 0:

      # invalid data, start tracking an outage
      if not self.in_outage[topic]:
        self.outage_start_t[topic] = t_msg
        self.outage_start_ts[topic].append(t_msg.to_sec())
        self.in_outage[topic] = True
    
    else:
      if self.in_outage[topic]:
        # outage just ended
        outage_duration = (t_msg - self.outage_start_t[topic]).to_sec()

        if outage_duration > self.MAX_OUTAGE_DURATION:
          self.found_anomaly('{} outage (> {}s)'.format(topic, self.MAX_OUTAGE_DURATION), self.outage_start_t[topic], t_msg, '')
          self.outage_durations[topic].append(outage_duration)

      self.in_outage[topic] = False
  
  def print_summary(self):

    # compute statistics...
    print(BOLD+'\nOutage Statistics'+RST)

    for topic in self.get_subscribed_channel():

      print('\n' + topic)

      if len(self.outage_durations[topic]) > 0:

        min_duration = min(self.outage_durations[topic])
        max_duration = max(self.outage_durations[topic])
        avg_duration = sum(self.outage_durations[topic]) / len(self.outage_durations[topic])

        squared_diffs = [pow(x - avg_duration, 2) for x in self.outage_durations[topic]]
        stddev_duration = math.sqrt(sum(squared_diffs) / len(squared_diffs))

        print(('Total Count: \t'+GRN+'{}'+RST).format(len(self.outage_durations[topic])))
        print(('Min Duration: \t'+GRN+'{:.3f}'+RST+' ms').format(min_duration * 1000))
        print(('Avg Duration: \t'+GRN+'{:.3f}'+RST+' ms').format(avg_duration * 1000))
        print(('Max Duration: \t'+GRN+'{:.3f}'+RST+' ms').format(max_duration * 1000))
        print(('Std Dev: \t'+GRN+'{:.3f}'+RST+' ms').format(stddev_duration * 1000))
      
      else:
        print(GRN+'(no outages detected)'+RST)
  
class LargeDeviationTest(AnomalyTest):

  # meters
  MAX_DEVIATION = 0.5

  def __init__(self):
    super(AnomalyTest,self).__init__()

    self.last_x = None
    self.last_y = None
    self.last_z = None
  
  def get_subscribed_channel(self):
    return ['SingleBaselineRTK']

  def feed_message(self, topic, msg, t_msg):

    # ignore invalid messages (otherwise we take lots of trips to the center of the earth...)
    if msg.bitfield & (1<<0) == 0:
      return

    if self.last_x is not None:
      dx = msg.rxRov - self.last_x
      dy = msg.ryRov - self.last_y
      dz = msg.rzRov - self.last_z

      dr = math.sqrt(dx*dx + dy*dy + dz*dz)

      if dr > self.MAX_DEVIATION:
        self.found_anomaly('large position step ({} m)'.format(dr), t_msg, t_msg, '')
    
    self.last_x = msg.rxRov
    self.last_y = msg.ryRov
    self.last_z = msg.rzRov
  
  def print_summary(self):
    # print('LargeDeviationTest ({} m)'.format(self.MAX_DEVIATION))
    pass

class DataGrapher(AnomalyTest):

  # meters
  MAX_DEVIATION = 0.5

  message_data = {}

  def __init__(self):
    super(AnomalyTest,self).__init__()

    for channel in self.get_subscribed_channel():
      self.message_data[channel] = {}
      self.message_data[channel]['t'] = []
      self.message_data[channel]['testStat'] = []
  
  def get_subscribed_channel(self):
    return ['SingleBaselineRTK','Attitude2D']

  def feed_message(self, topic, msg, t_msg):

    self.message_data[topic]['t'].append(t_msg.to_sec())
    self.message_data[topic]['testStat'].append(msg.testStat)
  
  def print_summary(self):
    # print('LargeDeviationTest ({} m)'.format(self.MAX_DEVIATION))
    pass

  def do_graph(self):

    for channel in self.get_subscribed_channel():
      plt.plot(self.message_data[channel]['t'], self.message_data[channel]['testStat'], label=channel)
      plt.plot(ExtendedOutageTest.outage_start_ts[channel], [200 for x in ExtendedOutageTest.outage_start_ts[channel]], 'o', label=channel+' outage')

    plt.xlabel('Unix Time (s)')
    plt.ylabel('TestStat')
    plt.title('TestStat')
    plt.legend()
    plt.show()

# ros timestamp: ns since unix epoch

def usage_message():
  print('Usage: anomalyze.py [rosbag.bag] [node_name]')

if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(description='Magic Anomaly Finder')

  parser.add_argument('-b', '--bag', metavar='bag_file', action='store', required=True, help='Read in data from a ROS bag file')
  parser.add_argument('-n', '--node', metavar='node_name', type=str, required=True, help='ROS node name')
  parser.add_argument('-g', '--graph', action='store_true', help='Graph statistics & anomaly data')

  args = parser.parse_args()

  bag_filename = args.bag
  node_name = args.node
  do_graph = args.graph

  print('Opening bag (this might take a while...)')
  bag = rosbag.Bag(bag_filename, skip_index=False)
  print('Analyzing messages...')

  testers = []

  for subclass in AnomalyTest.__subclasses__():
    instance = subclass()
    testers.append(instance)

  # Figure out which ros topics to pull messages from
  # (we only care about what the testers are subscribed to)
  subscribed_topics = []
  for tester in testers:
    new_topics = tester.get_subscribed_channel()
    for topic in new_topics:
      if topic not in subscribed_topics:
        subscribed_topics.append(topic)

  # Put together a thing to store message counts
  message_counts = {}

  for topic in subscribed_topics:
    message_counts[topic] = 0
  
  for topic,msg,t_msg in bag.read_messages(topics=['/{}/{}'.format(node_name, topic) for topic in subscribed_topics]):
    
    m = re.search('\/.+\/(.+)', topic)
    subtopic = m.group(1)

    # Pass this message to all testers that want it
    for tester in testers:
      if subtopic in tester.get_subscribed_channel():
        tester.feed_message(subtopic, msg, t_msg)
    
    message_counts[subtopic] += 1

    last_msg = msg

  print('Reached end of bag')

  for tester in testers:
    tester.print_summary()

  print('\nTotal messages examined:')
  for topic in subscribed_topics:
    print('\t/{}/{}: {}'.format(node_name, topic, message_counts[topic]))

  if do_graph:
    print('doing a graph')
    for tester in testers:
      tester.do_graph()

