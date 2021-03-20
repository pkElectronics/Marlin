import os
from os.path import join
from os.path import expandvars
Import("env")

env.AddPostAction(
	join("$BUILD_DIR","${PROGNAME}.elf"),
	env.VerboseAction(" ".join([
		"$OBJCOPY", "-O ihex", "$TARGET", 
		"\"" + join("$BUILD_DIR","${PROGNAME}.hex") + "\"", # Note: $BUILD_DIR is a full path
	]), "Building $TARGET"))

# In-line command with arguments
UPLOAD_TOOL="stm32flash"
platform = env.PioPlatform()
if platform.get_package_dir("tool-stm32duino") != None:
	UPLOAD_TOOL=expandvars("\"" + join(platform.get_package_dir("tool-stm32duino"),"stm32flash","stm32flash") + "\"")

env.Replace(
	UPLOADER=UPLOAD_TOOL,
	UPLOADCMD=expandvars(UPLOAD_TOOL + " -v -i '16,-17,17:-16,-17,17' -R -b 115200 -g 0 -w \"" + join("$BUILD_DIR","${PROGNAME}.hex")+"\"" + " /dev/serial0")
)


#env.Replace(
#    UPLOADCMD="stm32flash -v -g 0 -i ',,,,,GPIO_16,-GPIO_17,GPIO_17:,,,,,-GPIO_16,-GPIO_17,GPIO_17'  -R -w $BUILD_DIR/${PROGNAME}.hex"
#)