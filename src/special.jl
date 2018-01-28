#####################################################################
# special.jl
#
# includes non-standard code I needed for papers and whatnot 
######################################################################

function norm2(x::LocTuple, y::LocTuple)
	d1 = x[1] - y[1]
	d2 = x[2] - y[2]
	return sqrt(d1*d1 + d2*d2)
end

function sim(j::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false)
	# always start the vehicle in the center
	# NO, don't do that
	#x.x = m.length / 2.0
	#x.y = m.length / 2.0
	theta!(m, j[1], j[2])

	# warn user if jammer is not where it should be
	#if abs(dx) > x.x || abs(dy) > x.y
	#	println("WARNING: jammer outside search domain.")
	#end

	# loop through all observations...
	for (oi,o) in enumerate(observations)
		#update!(f, x, o)
		#plot(m, b[oi], x, show_mean=true, show_cov=true)
		plot(m, b[oi], x, show_mean=show_mean, show_cov=show_cov, obs=o)
		savefig("temp_$(oi).png", format="png")
		hold(false)
		act!(m, x, actions[oi])
	end
end

export sim2
function sim2(j::LocTuple, states::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false, show_path::Bool=true)
	# always start the vehicle in the center
	# NO, don't do that
	#x.x = m.length / 2.0
	#x.y = m.length / 2.0
	theta!(m, j[1], j[2])

	path_x = Float64[]
	path_y = Float64[]

	# loop through all observations...
	for (oi,o) in enumerate(observations)
		if show_path
			#plot path
			plot(path_x, path_y, "k")
		end
		hold(true)
		plot(m, b[oi], x, show_mean=show_mean, show_cov=show_cov, obs=oi)
		#savefig("temp_$(oi).pdf", format="pdf", dpi=300)
		savefig("temp_$(oi).png", format="png")
		hold(false)
		push!(path_x, x.x)
		push!(path_y, x.y)
		x.x = states[oi][1]
		x.y = states[oi][2]
		x.heading = states[oi][3]
		#act!(m, x, actions[oi])
	end
end

# This was for AIAA GNC 2018 paper.
# This probably shouldn't be here.
export eval2
function eval2(j::LocTuple, states::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false, show_path::Bool=true)
	# always start the vehicle in the center
	# NO, don't do that
	#x.x = m.length / 2.0
	#x.y = m.length / 2.0
	theta!(m, j[1], j[2])

    front_cone = Float64[]
    rear_cone = Float64[]
    sides = Float64[]


	# loop through all observations...
	for (oi,o) in enumerate(observations)
        # get true bearing between x
        xt = (x.x,x.y,x.heading)
        rel_bearing = fit_180(xt[3] - true_bearing(xt, m.theta))
        if rel_bearing < 0.0
            rel_bearing = -1.0 * rel_bearing
        end


        cla()
		plot(m, b[oi], x, show_mean=show_mean, show_cov=show_cov, obs=oi)
        #title("o = $o")
        #hold(false)
        if rel_bearing < 60.0
            # expect observation 1
            title("expect 1, got $o")
            push!(front_cone, o)
        elseif rel_bearing < 120.0
            title("expect either, got $o")
            push!(sides, o)
        else
            title("expect 0, got $o")
            push!(rear_cone,o)
        end
        pause(0.15)

        # update vehicle position
		x.x = states[oi][1]
		x.y = states[oi][2]
		x.heading = states[oi][3]
	end
    return front_cone, sides, rear_cone
end

function simulate(m::SearchDomain, uav::SimUnit;
                  video::Bool=true,
                  pause_time=0.3
                 )

    # reset the filter, vehicle, and policy
    # TODO: I think I assume the SimUnit comes in clean and ready to go
    #reset!(uav.f)
    #reset!(m, uav.x)
    #reset!(uav.p)

    # What was the cost to getting this first observation?
    temp_cost = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, uav.x)
    update!(uav, o)

    # This was our first step; steps count number of observations
    step_count = 1

    # plot if need be
    if video
        figure("Simulation")
        plot(m, uav.f, uav.x)
        title("i = $(step_count)")
    end


    while !is_complete(uav.f, uav.tc, step_count)
        # act
        a = action(m, uav, o)
        act!(m, uav.x, a)

        # get cost and update step count
        temp_cost += get_cost(uav, m, a)
        step_count += 1

        # observe and update
        o = observe(m, uav.x)
        update!(uav, o)

        # plot if need be
        if video
            pause(pause_time)
            figure("Simulation")
            cla()
            plot(m, uav.f, uav.x)
            max_b = maximum(uav.f.b)
            title("i = $(step_count), max = $(round(max_b,3))")
        end
    end

    return temp_cost
end
