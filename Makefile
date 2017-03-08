LIBS=wpi CTRLib navx_frc_cpp
override CFLAGS +=$(addprefix -l,$(LIBS)) -std=c++14
TEAM=1786
SSH_OPTIONS= -v
SSH_SSHPASS=$(shell command -v sshpass >/dev/null 2>&1 && echo -n "sshpass -p ''")

all: deploy

build: FRCUserProgram

FRCUserProgram: Robot.cpp
	@echo "Building FRCUserProgram"
	arm-frc-linux-gnueabi-g++ Robot.cpp -o FRCUserProgram $(CFLAGS)

clean:
	rm FRCUserProgram

deploy: build
	@echo "Copying FRCUserProgram"
	@ssh $(SSH_OPTIONS) lvuser@roborio-1786.local 'rm -f /home/lvuser/FRCUserProgram'
	@scp $(SSH_OPTIONS) -o "LogLevel QUIET" FRCUserProgram lvuser@roborio-1786.local:/home/lvuser/FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'

restart: FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'

stop:
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t'
