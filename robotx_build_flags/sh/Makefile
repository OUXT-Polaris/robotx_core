all: extract

clean:
	rm -rf catkin_ws.tar.gz

build: clean
	. arm.sh
	bash allatonce.sh

transfer: build
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
	terminal-notifier -title 'AWS Compile Finished' -message 'robotx AWS compile finished'
