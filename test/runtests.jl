#Test if tf listner works correctly
using RobotOS


# Initialize node
init_node("listner", anonymous=true) # delete

# Need to run the listener and broadcaster in parralllel
script = (`julia --project=@. tf_listener.jl` &
          `julia --project=@. tf_broadcaster.jl`);
Base.run(script);
