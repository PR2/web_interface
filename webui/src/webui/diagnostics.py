import roslib; roslib.load_manifest('webui')
import rospy
import sys
import rosweb
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class Status(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)

  def transform(self, msg):
    statuses = msg.status
    for status in statuses:
        if not status.name.startswith('/'):
            status.name = '/' + status.name
  
    def top_only(status): return status.name.count("/") < 2    
    filtered_statuses = filter(top_only, statuses)
    levels = [status.level for status in filtered_statuses]
    
    new_message = DiagnosticStatus()
    new_message.name = "Robot Status"
    new_message.level = max(levels)
    
    return new_message

class Summary(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)

  def transform(self, msg):
    statuses = msg.status
    for status in statuses:
        if not status.name.startswith('/'):
            status.name = '/' + status.name
  
    def top_only(status): return status.name.count("/") < 2    
    filtered_statuses = filter(top_only, statuses)

    #rospy.loginfo("statuses: %s" % statuses)
    #rospy.loginfo("f_statuses: %s" % filtered_statuses)

    #for status in filtered_statuses:
    #    rospy.loginfo("name: %s" % status.name)
    new_message = DiagnosticArray()
    new_message.status = filtered_statuses
    
    return new_message

class FilterDevice(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)
    _, _, self.params = rosweb.splitTopic(topic)
      
  def transform(self, msg):
    subtopic = self.params
    statuses = msg.status
    for status in statuses:
        if not status.name.startswith('/'):
            status.name = '/' + status.name
  
    def device(status): return status.name.startswith(subtopic)
    filtered_statuses = filter(device, statuses)
      
    new_message = DiagnosticArray()
    new_message.status = filtered_statuses
   
    return new_message

def config_plugin(context):
  context.register_subtopic("/diagnostics_agg:filter", FilterDevice)
  context.register_subtopic("/diagnostics_agg:Summary", Summary)
  context.register_subtopic("/diagnostics_agg:Status", Status)
