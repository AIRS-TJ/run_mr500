#! /usr/bin/python

import os
import rospy

import subprocess

from robuster_mr_msgs.srv import *


HOME_DIR = os.path.expanduser('~')
MAPS_DIR = HOME_DIR + '/.robuster/maps/'

## @brief used to manager system services
class ServiceManager:
    def __init__(self):
        self.mapping_started = False
        self.navigation_started = False
        self.service_list = []

        rospy.Service('/robuster/robuster_base/control', RobusterBaseControl, self.handleRobusterBaseControl)
        rospy.Service('/robuster/navigation/control', NavigationControl, self.handleNavigationControl)
        rospy.Service('/robuster/mapping/control', MappingControl, self.handleMappingControl)
        rospy.Service('/robuster/mapping/save', MappingSave, self.handleMappingSave)
        rospy.Service('/robuster/mapping/list', MappingList, self.handleMappingList)
        rospy.Service('/robuster/mapping/select', MappingSelect, self.handleMappingSelect)

    def addService(self, serviceName):
        # TODO: check if the service is exist or not
        self.service_list.append(serviceName)

    def startService(self, serviceName):
        if serviceName in self.service_list:
            p = subprocess.Popen(['sudo systemctl start '+serviceName+'.service'], shell=True, stdout=subprocess.PIPE)
            output, err = p.communicate()
            rospy.logdebug(output)
            return 0
        else:
            rospy.logwarn(serviceName + ".service doesn't exist.")
            return -1
    
    def stopService(self, serviceName):
        if serviceName in self.service_list:
            p = subprocess.Popen(['sudo systemctl stop '+serviceName+'.service'], shell=True, stdout=subprocess.PIPE)
            output, err = p.communicate()
            rospy.logdebug(output)
            return 0
        else:
            rospy.logwarn(serviceName + ".service doesn't exist.")
            return -1

    def handleRobusterBaseControl(self, req):
        if req.msg:
            result = self.startService('robuster_base')
            # TODO failed response
            return RobusterBaseControlResponse(result, "robuster_base start success.")
        else:
            result = self.stopService('robuster_base')
            # TODO failed response
            return RobusterBaseControlResponse(result, "robuster_base stop success.")

    def handleMappingControl(self, req):
        if self.navigation_started:
            return MappingControlResponse(-1, "Mapping Control failed because navigation is running.")
        else:
            if req.msg:
                self.mapping_started = True
                result = self.startService('mapping')
                # TODO failed response
                return MappingControlResponse(result, "Mapping start success.")
            else:
                self.mapping_started = False
                result = self.stopService('mapping')
                # TODO failed response
                return MappingControlResponse(result, "Mapping stop success.")
    
    def handleNavigationControl(self, req):
        if self.mapping_started:
            return NavigationControlResponse(-1, "Navigation Control failed because mapping is running.")
        else:
            if req.msg:
                self.navigation_started = True
                result = self.startService('navigation')
                return NavigationControlResponse(result, "Navigation start success.")
            else:
                self.navigation_started = False
                result = self.stopService('navigation')
                return NavigationControlResponse(result, "Navigation stop success.")

    def handleMappingSave(self, req):
        (filename, ext) = os.path.splitext(req.filename)
        if len(req.filename) > 0:
            if not os.path.exists(MAPS_DIR + filename):
                os.makedirs(MAPS_DIR + filename)
            else:
                return MappingSaveResponse(-1,'The map name already exists')

        filename = MAPS_DIR + filename + '/' + filename
        p = subprocess.Popen(['rosrun map_server map_saver --occ 55 --free 50 -f  '+ filename], shell=True, stdout=subprocess.PIPE)
        output, err = p.communicate()
        rospy.loginfo(output)
        return MappingSaveResponse(0,'Map save success.')
    
    def handleMappingSelect(self, req):
        if not self.navigation_started:
            rospy.loginfo("Select map file: " + req.filename)
            map_file = MAPS_DIR + req.filename + '/' + req.filename + '.yaml'
            rospy.set_param('selected_map', map_file)
            return MappingSelectResponse(0, 'Selected ' + req.filename)
        else:
            return MappingSelectResponse(-8, 'map select failed, navigation is running now.please stop navigation first')

    def handleMappingList(self, req):
        map_list = []
        dirs = [x[1] for x in os.walk(MAPS_DIR)]
        print(dirs)
        for dir in dirs:
            for subdir in dir:
                map_list.append(subdir)
                print('Get map: ' + subdir)

        return MappingListResponse(map_list, 0, 'Get map list success.') # {'names':map_list, 'status':0, 'message':'Get map list success.'}

if __name__ == '__main__':
    rospy.init_node('daemon_server', log_level=rospy.INFO)

    try:
        servs = ServiceManager()
        servs.addService('robuster_base')
        servs.addService('mapping')
        servs.addService('navigation')
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
