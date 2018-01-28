######################################################################
# gif.jl
#
# This file makes gifs of simulations
######################################################################

"""
`gif(m::SearchDomain, x::Vehicle, f::Filter, p::Policy, num_steps=10, filename=out.gif)`
"""
function gif(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, tc::TerminationCondition=StepThreshold(10), filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false)
	frames = Frames(MIME("image/png"), fps=20)
	path_x = Float64[]
	path_y = Float64[]

	# Plot the original scene
	plot(m, f, x)
    title("\$t\$ = 0 s")
	push!(frames, gcf())
	push!(path_x, x.x)
	push!(path_y, x.y)
	close()

	# 20 is the fps
	frames_per_step = round(Int, seconds_per_step * 20)

    step_count = 0
    while !is_complete(f, tc, step_count)
		old_pose = (x.x, x.y, x.heading)
		o = observe(m,x)
		update!(f, x, o)
		a = action(m, x, o, f, p)
		act!(m,x,a)
		new_pose = (x.x, x.y, x.heading)
		# Plot everything in between old pose and new pose
		dx = (new_pose[1] - old_pose[1]) / frames_per_step
		dy = (new_pose[2] - old_pose[2]) / frames_per_step
		dh = a[3] / frames_per_step
		for j = 1:frames_per_step
			#figure()
			# determine intermediate pose
			new_h = mod(old_pose[3] + dh, 360.0)
			old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)

			push!(path_x, old_pose[1])
			push!(path_y, old_pose[2])
			plot(m, f, old_pose;show_mean=show_mean, show_cov=show_cov)
			if show_path
				plot(path_x, path_y)
			end
            title("\$t\$ = $step_count s")
			push!(frames, gcf())
			close()
		end
        step_count += 1
	end
	#write(filename, frames)
	write("temp.mp4", frames)
	write("temp.gif", frames)
end

gif() = gif(Main.m, Main.x, Main.f, Main.p, 10)


function gif{TF<:AbstractFilter, TP<:Policy}(m::SearchDomain, xarr::Vector{Vehicle}, farr::Vector{TF}, parr::Vector{TP}, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false, alpha=1.0)
	num_vehicles = length(xarr)
	obsarr = zeros(num_vehicles)
	new_poses = Array(Pose, num_vehicles)
	old_poses = Array(Pose, num_vehicles)
	diffs = Array(Pose, num_vehicles)

	frames = Frames(MIME("image/png"), fps=20)

	# Set up the paths
	x_paths = Array(Vector{Float64}, num_vehicles)
	y_paths = Array(Vector{Float64}, num_vehicles)
	for (xi,x) in enumerate(xarr)
		x_paths[xi] = [x.x]
		y_paths[xi] = [x.y]
	end

	# Plot the original scene
	plot(m, farr, xarr)
	push!(frames, gcf())
	close()

	# 20 is the fps
	frames_per_step = round(Int, seconds_per_step * 20)

	colors = ["b", "r"]
	if num_vehicles == 1
		colors = ["b"]
	elseif num_vehicles == 2
		colors = ["b", "r"]
	elseif num_vehicles == 3
		colors = ["b", "r", "g"]
	elseif num_vehicles == 4
		colors = ["b", "r", "g", "k"]
	end

	for i = 1:num_steps
		for (xi,x) in enumerate(xarr)
			f = farr[xi]
			p = parr[xi]
			old_pose = (x.x, x.y, x.heading)
			old_poses[xi] = old_pose
			o = observe(m,x)
			update!(f, x, o)
			a = action(m, x, o, f, p)
			act!(m,x,a)
			new_pose = (x.x, x.y, x.heading)
			new_poses[xi] = new_pose
			# Plot everything in between old pose and new pose
			dx = (new_pose[1] - old_pose[1]) / frames_per_step
			dy = (new_pose[2] - old_pose[2]) / frames_per_step
			dh = a[3] / frames_per_step
			diffs[xi] = (dx,dy,dh)
		end
		for j = 1:frames_per_step
			#figure()
			# determine intermediate pose
			for (xi, x) in enumerate(xarr)
				dx = diffs[xi][1]
				dy = diffs[xi][2]
				dh = diffs[xi][3]
				old_pose = old_poses[xi]
				new_h = mod(old_pose[3] + dh, 360.0)
				old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)
				old_poses[xi] = (old_pose)

				push!(x_paths[xi], old_pose[1])
				push!(y_paths[xi], old_pose[2])
			end
			plot(m, farr, old_poses; show_mean=show_mean, show_cov=show_cov, alpha=alpha)
			if show_path
				for (xi, x) in enumerate(xarr)
					plot(x_paths[xi], y_paths[xi], colors[xi])
				end
			end
			push!(frames, gcf())
			close()
		end
	end
	write(filename, frames)
end

export simgif

# really, this is for white sands results
function simgif(j::LocTuple, actions::Vector{Pose}, observations::Vector{Float64}, b::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false, show_path=false)
	# always start the vehicle in the center
	# NO, don't do that
	#x.x = m.length / 2.0
	#x.y = m.length / 2.0
	theta!(m, j[1], j[2])

	num_vehicles = 1
	new_poses = Array(Pose, num_vehicles)
	old_poses = Array(Pose, num_vehicles)
	diffs = Array(Pose, num_vehicles)

	frames = Frames(MIME("image/png"), fps=20)

	# Set up the paths
	x_path = [x.x]
	y_path = [x.y]

	# Plot the original scene
	#plot(m, b[1], x, show_mean=show_mean, show_cov=show_cov, obs=observations[1])
	plot(m, b[1], x, show_mean=show_mean, show_cov=show_cov)
	push!(frames, gcf())
	close()

	# 20 is the fps
	seconds_per_step = 2.0
	frames_per_step = round(Int, seconds_per_step * 20)

	# warn user if jammer is not where it should be
	#if abs(dx) > x.x || abs(dy) > x.y
	#	println("WARNING: jammer outside search domain.")
	#end
	colors = ["b", "r"]
	if num_vehicles == 1
		colors = ["b"]
	elseif num_vehicles == 2
		colors = ["b", "r"]
	elseif num_vehicles == 3
		colors = ["b", "r", "g"]
	elseif num_vehicles == 4
		colors = ["b", "r", "g", "k"]
	end

	# loop through all observations...
	num_obs = length(observations)
	for (oi,o) in enumerate(observations)
		println("oi = ", oi)
		xi = 1
		old_pose = (x.x, x.y, x.heading)
		old_poses[xi] = old_pose

		a = actions[oi]
		act!(m, x, actions[oi])
		new_pose = (x.x, x.y, x.heading)
		new_poses[xi] = new_pose
		# Plot everything in between old pose and new pose
		dx = (new_pose[1] - old_pose[1]) / frames_per_step
		dy = (new_pose[2] - old_pose[2]) / frames_per_step
		dh = a[3] / frames_per_step
		diffs = (dx, dy, dh)

		#plot(m, b[oi], x, show_mean=true, show_cov=true)
		#println("oi = ", oi)
		FEBOL.plot(m, b[oi], x, show_mean=show_mean, show_cov=show_cov)
		for j = 1:frames_per_step
			xi = 1
			figure()
			# determine intermediate pose
			dx, dy, dh = diffs
			old_pose = old_poses[xi]
			new_h = mod(old_pose[3] + dh, 360.0)
			old_pose = (old_pose[1] + dx, old_pose[2] + dy, new_h)
			old_poses[xi] = (old_pose)
			push!(x_path, old_pose[1])
			push!(y_path, old_pose[2])
			#plot(m, farr, old_poses; show_mean=show_mean, show_cov=show_cov, alpha=alpha)
			#FEBOL.plot(m, b[oi], old_pose, show_mean=show_mean, show_cov=show_cov, obs=o)
			FEBOL.plot(m, b[oi], old_pose, show_mean=show_mean, show_cov=show_cov)
			if show_path
				plot(x_path, y_path, colors[1])
			end

			push!(frames, gcf())
			close()
		end
		#savefig("temp_$(oi).png", format="png")
		#hold(false)
		#act!(m, x, actions[oi])
	end
	filename="temp.gif"
	write(filename, frames)
end

# really, this is for white sands results
# This shouldn't be so hard...
export simgif2
function simgif2(states::Vector{Pose}, beliefs::Vector{Matrix{Float64}}, m::SearchDomain, x::Vehicle; show_mean=false, show_cov=false, show_path=false, j::LocTuple=m.theta)

	theta!(m, j[1], j[2])
	frames_per_second = 4
	frames = Frames(MIME("image/png"), fps=frames_per_second)

	# Set up the paths
	x_path = [x.x]
	y_path = [x.y]

	# Plot the original scene
	plot(m, beliefs[1], x, show_mean=show_mean, show_cov=show_cov,obs=0)
	push!(frames, gcf())
	close()

	# 20 is the fps
	seconds_per_step = 1.0
	frames_per_step = round(Int, seconds_per_step * frames_per_second)

	colors = ["b"]

	# loop through all beliefs...
	num_b = length(beliefs)
	s = states[1]
	for bi = 2:(num_b+1)
		println("bi = ", bi)
		sp = states[num_b]
		if bi < (num_b + 1)
			sp = states[bi]
		end

		# Plot everything in between old pose and new pose
		dx = (sp[1] - s[1]) / frames_per_step
		dy = (sp[2] - s[2]) / frames_per_step
		dh = fit_180(sp[3] - s[3]) / frames_per_step
		diffs = (dx, dy, dh)

		#plot(m, b[oi], x, show_mean=true, show_cov=true)
		#println("oi = ", oi)
		#FEBOL.plot(m, beliefs[bi-1], x,show_mean=show_mean, show_cov=show_cov)
		for j = 1:frames_per_step
			figure()
			# determine intermediate pose
			dx, dy, dh = diffs
			new_h = mod(s[3] + dh, 360.0)
			s = (s[1] + dx, s[2] + dy, new_h)

			push!(x_path, s[1])
			push!(y_path, s[2])

			FEBOL.plot(m, beliefs[bi-1], s, show_mean=show_mean, show_cov=show_cov, obs=bi-1)
			if show_path
				# changed by LD for rss submission
				#plot(x_path, y_path, colors[1])
				plot(x_path, y_path, "k")
			end

			push!(frames, gcf())
			close()
		end
	end
	#filename="temp.mp4"
	filename="temp.gif"
	write(filename, frames)
end
