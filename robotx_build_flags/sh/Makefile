update: clean update-aws transfer extract
build: clean build-aws transfer extract

clean:
	rm -rf catkin_ws.tar.gz

build-aws:
	. arm.sh && bash allatonce.sh

update-aws:
	. arm.sh && bash allatonce-update.sh

transfer:
	scp catkin_ws.tar.gz aibox1:/home/nvidia/
	scp catkin_ws.tar.gz aibox2:/home/nvidia/
	scp catkin_ws.tar.gz aibox3:/home/nvidia/

transfer-aibox1:
	scp catkin_ws.tar.gz aibox1:/home/nvidia/

transfer-aibox2:
	scp catkin_ws.tar.gz aibox2:/home/nvidia/

transfer-aibox3:
	scp catkin_ws.tar.gz aibox3:/home/nvidia/

extract: transfer
	ssh aibox1 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
	ssh aibox2 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
	ssh aibox3 'cd /home/nvidia/ && tar xf catkin_ws.tar.gz'
