MS5803_14BA_DIR=MS5803_14BA
GLUEDIR=glue
EMPLDIR = eMPL
GLUEDIR = glue
MPU9150DIR = MPU9150
OBJDIR=obj
AHRSDIR = AHRS

INCLUDES=-I $(MS5803_14BA_DIR) -I $(GLUEDIR) -I $(EMPLDIR) -I $(MPU9150DIR) -I $(AHRSDIR)

# add -DI2C_DEBUG for debugging
DEFS = -DEMPL_TARGET_LINUX -DMPU9150

OBJ= 	$(OBJDIR)/MS5803_14BA.o \
		$(OBJDIR)/mpu9150.o \
		$(OBJDIR)/linux_glue.o \
		$(OBJDIR)/inv_mpu.o \
		$(OBJDIR)/inv_mpu_dmp_motion_driver.o \
		$(OBJDIR)/AHRS.o \
		$(OBJDIR)/piuw.o

		 
piuw: $(OBJ)
	g++ -o piuw $(OBJ) $(INCLUDES)

$(OBJDIR)/piuw.o: piuw.cpp
	g++ -c piuw.cpp $(INCLUDES) -o $(OBJDIR)/piuw.o

$(OBJDIR)/linux_glue.o: $(GLUEDIR)/linux_glue.c $(GLUEDIR)/linux_glue.h
	gcc -c $(GLUEDIR)/linux_glue.c $(INCLUDES) $(DEFS) -o $(OBJDIR)/linux_glue.o 
	
$(OBJDIR)/inv_mpu.o: $(EMPLDIR)/inv_mpu.h $(EMPLDIR)/inv_mpu.c
	gcc -c $(EMPLDIR)/inv_mpu.c $(INCLUDES) $(DEFS) -o $(OBJDIR)/inv_mpu.o  
	
$(OBJDIR)/inv_mpu_dmp_motion_driver.o: $(EMPLDIR)/inv_mpu_dmp_motion_driver.h $(EMPLDIR)/inv_mpu_dmp_motion_driver.c
	gcc -c $(EMPLDIR)/inv_mpu_dmp_motion_driver.c $(DEFS) $(INCLUDES) -o $(OBJDIR)/inv_mpu_dmp_motion_driver.o  
	
$(OBJDIR)/mpu9150.o: $(MPU9150DIR)/mpu9150.h $(MPU9150DIR)/mpu9150.cpp
	g++ -c $(MPU9150DIR)/mpu9150.cpp $(INCLUDES) $(DEFS) -o $(OBJDIR)/mpu9150.o  
		
$(OBJDIR)/MS5803_14BA.o: $(MS5803_14BA_DIR)/MS5803_14BA.h $(MS5803_14BA_DIR)/MS5803_14BA.cpp
	g++ -c $(MS5803_14BA_DIR)/MS5803_14BA.cpp $(INCLUDES) $(DEFS) -o $(OBJDIR)/MS5803_14BA.o  
	
$(OBJDIR)/AHRS.o: $(AHRSDIR)/AHRS.h  $(AHRSDIR)/AHRS.cpp
	g++ -c $(AHRSDIR)/AHRS.cpp $(INCLUDES) $(DEFS) -o $(OBJDIR)/AHRS.o
	
clean: 
	rm $(OBJ) piuw
