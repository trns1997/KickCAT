add_executable(eoe_example eoe_example.cc)
target_link_libraries(eoe_example kickcat argparse::argparse)
set_kickcat_properties(eoe_example)
