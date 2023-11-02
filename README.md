# mech_423_lab3
## Message Packet
### Commands
| cmdByte | description              | acceptable data | MCU pin(s)          |
|---------|--------------------------|-----------------|---------------------|
|  0      | DC Motor STOP            | ...             | P2.1                |
|  1      | DC Motor CW  Duty Cycle  | 0 to 65535      | P2.1                |
|  2      | DC Motor CCW Duty Cycle  | 0 to 65535      | P2.1                |

### Example Messages
| desc     | BYTE| cmdByte| data_H_Byte | data_L_Byte | escByte | data_modified |
|----------|-----|--------|-------------|-------------|---------|---------------|
| E-STOP   | 255 | 0      | ...         | ...         | ...     | ...           |
| DC, CW   | 255 | 1      | 128         | 0           | 0       | 32768 -> 50%  |
| DC, CW   | 255 | 1      |  64         | 0           | 0       | 32768 -> 25%  |
| DC, CCW  | 255 | 2      | 128         | 0           | 0       | 32768 -> 50%  |
| DC, CCW  | 255 | 2      |  64         | 0           | 0       | 32768 -> 25%  |

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
| cmdByte | description              | data (duty cycle) | P2.1 | P3.7 | P3.6  |
|---------|--------------------------|-------------------|------|------|-------|
|  1      | DC Motor CW  Duty Cycle  | 0 to 65535        | PWM  | 0    | 1     |
|  2      | DC Motor CCW Duty Cycle  | 0 to 65535        | PWM  | 1    | 0     |

### Ex3
| cmdByte | description              | data (duty cycle) | P2.1 | P3.7 | P3.6  |
|---------|--------------------------|-------------------|------|------|-------|
|  3      | Stepper  STOP            |                   | PWM  | 0    | 1     |
|  4      | Stepper  CW  Half Step   |                   | PWM  | 0    | 1     |
|  5      | Stepper  CCW Half Step   |                   | PWM  | 0    | 1     |
|  6      | Stepper  CW  Duty Cycle  | 0 to 65535        | PWM  | 0    | 1     |
|  7      | Stepepr  CCW Duty Cycle  | 0 to 65535        | PWM  | 1    | 0     |

| port | PBC       | color | stepper wire | Timer |
|------|-----------|-------|--------------|-------|
| P1.5 | AIN1_DRV1 | BLACK | A            | TB0.2 |
| P1.4 | AIN2_DRV1 | GREEN | C            | TB0.1 |
| P3.5 | BIN1_DRV1 | RED   | B            | TB1.1 |
| P3.4 | BIN2_DRV1 | BLUE  | D            | TB1.2 |

#### Half Stepping Table (from class)
| step | B | D | A | C |
|------|---|---|---|---|
| 0    | 1 | 0 | 0 | 0 |
| 1    | 1 | 0 | 1 | 0 |
| 2    | 0 | 0 | 1 | 0 |
| 3    | 0 | 1 | 1 | 0 |
| 4    | 0 | 1 | 0 | 0 |
| 5    | 0 | 1 | 0 | 1 |
| 6    | 0 | 0 | 0 | 1 |
| 7    | 1 | 0 | 0 | 1 |

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
