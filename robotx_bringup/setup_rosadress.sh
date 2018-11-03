#sshpass -p 'nvidia' ssh nvidia@192.168.1.101
#rosaddress client 192.168.1.101
#exit
sshpass -p 'nvidia' ssh nvidia@192.168.1.103
bash
roscd robotx_bringup
source rosaddress.bash
rosaddress server
exit