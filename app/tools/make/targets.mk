CXX_COMMAND=$(CXX) $(CFLAGS) -c $< -o $(BIN)/$@
CXX_COMMAND_SILENT="  CXX   $@"
.cpp.o:
	@$(if $(QUIET), ,echo $(CXX_COMMAND$(VERBOSE)) )
	@$(CXX_COMMAND)

include $(CRAZYFLIE_BASE)/tools/make/targets.mk