# See ESF-RAW

uint32 data      # Its scaling and unit depends on the type and is
                # the same as in ESF-MEAS
uint32 DATA_FIELD_MASK = 16777215
uint32 DATA_TYPE_MASK = 4278190080    # type of data 
                                      # (0 = no data; 1..255 = data type)
uint32 sTtag     # sensor time tag