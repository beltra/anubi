#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/${ROS_DISTRO}/setup.bash
source /tools/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
#                             <NetworkInterface name="enx3c18a099eeb4" priority="default" multicast="default" />
#                         </Interfaces></General>
#                         <Internal>
#                             <SocketReceiveBufferSize min="10MB"/>
#                             <Watermarks>
#                                 <WhcHigh>5MB</WhcHigh>
#                             </Watermarks>
#                         </Internal></Domain></CycloneDDS>'

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enx3c18a099eeb4" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
