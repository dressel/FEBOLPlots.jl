######################################################################
# gps.jl
# handles some gps functions
######################################################################

using GPS2XY

export gps_sim, gps_offset
export parse_log
export parse_belief, parse_action


# create simulation and drawing from
# j is the jammer start point (lat, lon) decimal gps coords
# x0 is the uav start point (lat, lon) decimal gps coords
function gps_sim(j::LocTuple, x0::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, m::SearchDomain, x::Vehicle, f::AbstractFilter; show_mean::Bool=false, show_cov::Bool=false)

	# always start the vehicle in the center
	x.x = m.length / 2.0
	x.y = m.length / 2.0
	dx,dy = gps_offset(x0, j)
	theta!(m, (x.x+dx, x.y+dy))

	# warn user if jammer is not where it should be
	if abs(dx) > x.x || abs(dy) > x.y
		println("WARNING: jammer outside search domain.")
	end

	# loop through all observations...
	for (oi,o) in enumerate(observations)
		update!(f, x, o)
		plot(m, f, x,show_mean=show_mean,show_cov=show_cov)
		savefig("temp_$(oi).png", format="png")
		hold(false)
		act!(m, x, actions[oi])
	end
end

function gps_sim(j::LocTuple, x0::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false)

	# always start the vehicle in the center
	x.x = m.length / 2.0
	x.y = m.length / 2.0
	dx,dy = gps_offset(x0, j)
	theta!(m, (x.x+dx, x.y+dy))

	# warn user if jammer is not where it should be
	if abs(dx) > x.x || abs(dy) > x.y
		println("WARNING: jammer outside search domain.")
	end

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


# This is not really gps related
# Reads my log file
# TODO: handle different types of beliefs
function parse_log(filename::String)

    obs = Float64[]
    actions = Pose[]
    beliefs = Array{Matrix{Float64}}(0)
	#states = Array(Pose, 0)
    states = Pose[]

	logfile = open(filename, "r")

	# create the filter and stuff
	line = readline(logfile)
	while !contains(line, "#####")
		# read header stuff

		line = readline(logfile)
	end

	# now get the actions, observations and beliefs
	while (line = readline(logfile)) != ""
		if contains(line, "observation")
			# observation line
			push!(obs, parse(Float64, split(line, " = ")[2]) )
		elseif contains(line, "action")
			# action line
			a_vec = split(split(line, " = ")[2], ",")
			# the actions we get are north,east,yaw. we want dx,dy,yaw
			dx  = parse(Float64, a_vec[2])
			dy  = parse(Float64, a_vec[1])
			dh  = parse(Float64, a_vec[3])
			push!(actions, (dx, dy, dh))
		elseif contains(line, "belief")
			# now read the next bunch of lines
			bline = readline(logfile)
			arr = split(bline, ",")
			n = length(arr)
            b = zeros(n,n)
			for i = 1:n
				b[i,n] = parse(Float64, arr[i])
			end
			for i = 2:n
				bline = readline(logfile)
				arr = split(bline, ",")
				for j = 1:n
					b[j,n-i+1] = parse(Float64, arr[j])
				end
			end
			push!(beliefs, b)
		elseif contains(line, "state")
			# action line
			s_vec = split(split(line, " = ")[2], ",")
			# the actions we get are north,east,yaw. we want dx,dy,yaw
			sx  = parse(Float64, s_vec[2])
			sy  = parse(Float64, s_vec[1])
			sh  = parse(Float64, s_vec[3])
			push!(states, (sx, sy, sh))
		end
	end
	close(logfile)

	return obs,actions,beliefs,states
end

# I'm not sure this would work...
function parse_belief(logfile)
	# now read the next bunch of lines
	bline = readline(logfile)
	if bline == ""
		error("No belief here.")
	end
	arr = split(bline, ",")
	n = length(arr)
	b = Array(Float64, n, n)
	for i = 1:n
		b[i,n] = parse(Float64, arr[i])
	end
	for i = 2:n
		bline = readline(logfile)
		arr = split(bline, ",")
		for j = 1:n
			b[j,n-i+1] = parse(Float64, arr[j])
		end
	end

	return b
end

"""
`parse_action(logfile)`

Returns tuple `(d_east,d_north,d_yaw)`
"""
function parse_action(logfile)
	line = readline(logfile)
	a_vec = split(split(line, " = ")[2], ",")
	# the actions we get are north,east,yaw. we want dx,dy,yaw
	d_east  = parse(Float64, a_vec[2])
	d_north  = parse(Float64, a_vec[1])
	d_yaw  = parse(Float64, a_vec[3])
	return d_east, d_north, d_yaw
end

# Meh, I don't know if I will use this...
function parse_obs(logfile)
	line = readline(logfile)
	if line == ""
		error("NO OBS HERE")
	end
	d_yaw  = parse(Float64, a_vec[3])
	return 1
end
