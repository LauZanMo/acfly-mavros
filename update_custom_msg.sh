#!/bin/bash

# 请在有acfly-mavros和mavlink的工作空间下执行该脚本
cp src/acfly-mavros/*.xml src/mavlink/message_definitions/v1.0/
catkin clean --yes
catkin build