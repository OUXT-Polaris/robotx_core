SHELL := /bin/bash
SCP   := scp -i wamv.pem
SSH   := ssh -i wamv.pem
aibox1 := nvidia@192.168.1.101
aibox2 := nvidia@192.168.1.102
aibox3 := nvidia@192.168.1.103

update: clean update-aws transfer extract
	echo 'updating'

build: clean build-aws transfer extract
	echo 'building'

clean:
	rm -rf catkin_ws.tar.gz

build-aws:
	source arm.sh && bash allatonce.sh

update-aws:
	source arm.sh && bash allatonce-update.sh

transfer:
	$(SCP) catkin_ws.tar.gz $(aibox1):~
	$(SCP) catkin_ws.tar.gz $(aibox2):~
	$(SCP) catkin_ws.tar.gz $(aibox3):~

transfer-aibox1:
	$(SCP) catkin_ws.tar.gz $(aibox1):~

transfer-aibox2:
	$(SCP) catkin_ws.tar.gz $(aibox2):~

transfer-aibox3:
	$(SCP) catkin_ws.tar.gz $(aibox3):~

extract: transfer
	$(SSH) $(aibox1) 'cd && tar xf catkin_ws.tar.gz'
	$(SSH) $(aibox2) 'cd && tar xf catkin_ws.tar.gz'
	$(SSH) $(aibox3) 'cd && tar xf catkin_ws.tar.gz'
