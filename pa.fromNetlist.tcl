
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name OMID2 -dir "D:/New folder/OMID2/planAhead_run_4" -part xc6slx9tqg144-3
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "D:/New folder/OMID2/Robot.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {D:/New folder/OMID2} }
set_property target_constrs_file "robot.ucf" [current_fileset -constrset]
add_files [list {robot.ucf}] -fileset [get_property constrset [current_run]]
link_design
