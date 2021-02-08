# TwizyProject
#### Github repository for bachelor thesis

Starting the car requiers the correct codes on both micro controller in in the front of the car,
contact the group members from 2018 for more info. There are some elctrical problems in the car 
(from previus projects), if the car dashnoard says "stop" the car will not drive and needs a restart 
if it does not go away. After every run the power cabel (USB connecting on the left side)for the
throttle will need to be disconnected, otherwise it will store last value and drive unwanted. 

To run with wasd
0 Connect laptop to CAN
1 Run roscore
2 Run rosrun twizy control_node
3 Run rosrun twizy testreader.py


### För att ladda ner denna github till egen dator ###
0 ~ cd catkin_ws
1 ~ git clone https://github.com/CapstenUnder/twizy.git
2 follow the guide for downloading Canlib to linux https://www.kvaser.com/canlib-webhelp/page_installing.html
3 när du är i din catkin_ws/src ~ catkin_make 

### För att slippa att skriva in "source ~/catkin_ws/devel/setup.bash" varenda jävla gång i ny terminal ###
1 ~ nano ~/.bashrc
2 skrolla lägst ner i dokumentet som kommer upp och lägg in raden "source ~/catkin_ws/devel/setup.bash"
3 klicka på ctr + o
4 klicka på enter
5 klicka på crt + x

### GLAD PÅSK ###
