# mech_423_lab3
## Message Packet
### Commands
| cmdByte | description              | acceptable data | MCU pin(s)          |
|---------|--------------------------|-----------------|---------------------|
|  1      | DC Motor CW  Duty Cycle  | 0 to 65535      | P2.1                |
|  2      | DC Motor CCW Duty Cycle  | 0 to 65535      | P2.1                |

### Example Messages
| MSG_START_BYTE | cmdByte| data_H_Byte | data_L_Byte | escByte | data_modified |
|----------------|--------|-------------|-------------|---------|---------------|
| 255            | 1      | 128         | 0           | 0       | 32768 -> 50%  |
| 255            | 1      |  64         | 0           | 0       | 32768 -> 25%  |
| 255            | 2      | 128         | 0           | 0       | 32768 -> 50%  |
| 255            | 2      |  64         | 0           | 0       | 32768 -> 25%  |

## MCU IO Pin Mapping
### Motor Driver
P3.7 -> AIN1

P3.6 -> AIN2

### Timer
TB2.1 -> P2.1

### UART
UCA0TXD -> P2.0

UCA0RXD -> P2.1

## Documentation
### Ex2
 0 -> STOP       | P3.7-> 0, P3.6-> 0 
 1 ->  CW        | P3.7-> 1, P3.6-> 0 
 2 -> CCW        | P3.7-> 0, P3.6-> 1 

## Set Up
### CCS Project
New -> CCS Project -> MSP430FR5739

Create a project for ex 2 to 6

### GitBash
```
mkdir l3
git clone git@github.com:slin14/mech_423_lab3.git l3github
cd l3github
export REPO=`pwd`
cd ../l3
export CCS=`pwd`
cd $REPO
for i in {2..6};
do
  ln -s $CCS/ex$i/main.c ex$i.c;
done
```
### Copy header to CCS Workspace
```
for i in {2..6};
do
  cp $REPO/mech423.h $CCS/ex$i/;
  cp $REPO/mech423PCB.h $CCS/ex$i/;
done
```
