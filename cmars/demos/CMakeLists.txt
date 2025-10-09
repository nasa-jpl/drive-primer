set(INCLUDE_DIRS "../src/")


set(DEMO_NAMES 
	# rigid_rig 
	# sol_01294
	cmars
	slipslope
)

foreach(demo ${DEMO_NAMES})
	add_executable(demo_${demo} ${demo}.cpp)
	target_compile_definitions(demo_${demo} PRIVATE "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 

	if(MSVC)
		set_target_properties(demo_${demo} PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
	endif()
	target_link_libraries(demo_${demo}  PRIVATE  ${CHRONO_TARGETS} ${IMAGE_DATA_LIB})
	target_include_directories(demo_${demo} PUBLIC ${INCLUDE_DIRS}  ${IMAGE_DATA_INCLUDES})

	#	add_DLL_copy_command()

endforeach()
