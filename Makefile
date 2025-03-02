obj-m := stepper_motor.o

# stepper_motor-y := stepper_motor.o

KDIR := /lib/modules/$(shell uname -r)/build/


all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
