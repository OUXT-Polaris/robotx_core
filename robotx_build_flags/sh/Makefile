all: clean catkin_ws.tar.gz transfer

clean:
	rm -rf catkin_ws.tar.gz

catkin_ws.tar.gz: clean
	. arm.sh
	bash allatonce.sh

transfer: catkin_ws.tar.gz
	scp catkin_ws.tar.gz aibox1:/home/nvidia/
	scp catkin_ws.tar.gz aibox2:/home/nvidia/
	scp catkin_ws.tar.gz aibox3:/home/nvidia/

extract: transfer
	ssh aibox1 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
	ssh aibox2 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
	ssh aibox3 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
