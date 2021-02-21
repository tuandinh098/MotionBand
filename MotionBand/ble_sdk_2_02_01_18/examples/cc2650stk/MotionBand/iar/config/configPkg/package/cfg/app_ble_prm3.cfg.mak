# invoke SourceDir generated makefile for app_ble.prm3
app_ble.prm3: .libraries,app_ble.prm3
.libraries,app_ble.prm3: package/cfg/app_ble_prm3.xdl
	$(MAKE) -f E:\5_Working_FIOT\F2_MotionBand\MotionBand_ver2\Source\MotionBand\ble_sdk_2_02_01_18\examples\cc2650stk\MotionBand\iar\config/src/makefile.libs

clean::
	$(MAKE) -f E:\5_Working_FIOT\F2_MotionBand\MotionBand_ver2\Source\MotionBand\ble_sdk_2_02_01_18\examples\cc2650stk\MotionBand\iar\config/src/makefile.libs clean

