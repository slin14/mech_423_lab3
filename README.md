# mech_423_lab3
## Message Packet
 | MSG_START_BYTE | cmdByte| data_H_Byte | data_L_Byte | escByte | data_modified |
 |----------------|--------|-------------|-------------|---------|---------------|
 | 255            | 1      | 128         | 0           | 0       | 32768 -> 50%  |
 | 255            | 1      |  64         | 0           | 0       | 32768 -> 25%  |
 | 255            | 2      |   0         | 0           | 0       | 0     -> STOP |
 | 255            | 2      |   0         | 1           | 0       | 1     ->  CW  |
 | 255            | 2      |   0         | 2           | 0       | 2     -> CCW  |

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
