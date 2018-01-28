######################################################################
# plotting.jl
# Handles all the calls to PyPlot to generate plots
#
# hold is deprecated as of Matplotlib 2.0
#  Figures now default to hold (true)
#  I think we should adhere to this for future compliance
######################################################################

"""
`plot(m::SearchDomain, f::AbstractFilter, x::Vehicle)`

Plots the belief, jammer, and vehicles.
"""
function plot(m::SearchDomain, f::AbstractFilter, x::Vehicle; show_mean::Bool=false, show_cov::Bool=false, alpha=1.0, color="b")
	plot(m, f, (x.x, x.y, x.heading); show_mean=show_mean, show_cov=show_cov, alpha=alpha, color=color)
end
# allow plotting of multiple vehicles with a single shared filter
function plot(m::SearchDomain, f::AbstractFilter, vx::Vector{Vehicle})
	plot(m, f, vx[1])
	num_vehicles = length(vx)
	for i = 2:num_vehicles
		plot_vehicle(m, vx[i])
	end
end
function plot(m::SearchDomain, f::AbstractFilter, p::Pose; show_mean::Bool=false, show_cov::Bool=false, alpha=1.0, color="b")
	plot_theta(m)
	#hold(true) # deprecated
	plot_vehicle(m, p; color=color)
	if show_mean
		plot_mean(centroid(f), color=color)
	end
	if show_cov
		plot(m, centroid(f), covariance(f), color=color)
	end
	plot(m, f, alpha=alpha)
	return # so it doesn't spit out result of axis
end

function plot(m::SearchDomain, b::Matrix{Float64}, x::Vehicle; show_mean::Bool=false, show_cov::Bool=false, alpha=1.0, color="b", obs=nothing)
	plot(m, b, (x.x, x.y, x.heading); show_mean=show_mean, show_cov=show_cov, alpha=alpha, color=color, obs=obs)
end

function plot(m::SearchDomain, b::Matrix{Float64}, p::Pose; show_mean::Bool=false, show_cov::Bool=false, alpha=1.0, color="b", obs=nothing)
	plot_theta(m)
	#hold(true)  # deprecated
	plot_vehicle(m, p; color=color)
	#title_string = "plot: "
	title_string = ""
	if obs != nothing
		# TODO: just temporary for rss submission
		#title_string = "$(title_string)o = $(obs), "
		title_string = "t = $(obs) s"
	end
	if show_mean
		plot_mean(centroid(b, m.length), color=color)
	end
	if show_cov
		cov = covariance(b,m.length)
		evals = eigvals(cov)
		e1 = round(evals[1],1)
		e2 = round(evals[2],1)
		plot(m, centroid(b,m.length), cov, color=color)
		title_string = "$(title_string)e1 = $e1, e2 = $e2"
		#title("e1 = $e1, e2 = $e2")
	end
	title(title_string, fontname="Times New Roman", fontsize=12)
	plot(m, b, alpha=alpha)
	return # so it doesn't spit out result of axis
end

function plot(m::SearchDomain, f::AbstractFilter; alpha=1.0)
	error(typeof(f), " has not implemented `plot`(::SearchDomain, ::AbstractFilter)")
end

function plot(m::SearchDomain, f::DF; alpha=1.0, cmap="Greys")
	plot(m, f.b, alpha=alpha, cmap=cmap)
end
function plot(m::SearchDomain, b::Matrix{Float64}; alpha=1.0, cmap="Greys")
	# alpha changed by LD for rss submission 2017
	alpha = 0.5
	alpha = 0.6
	a = [0,m.length,0,m.length]
	imshow(b', interpolation="none",cmap=cmap,origin="lower",extent=a,vmin=0, alpha=alpha)
	labels()
	axis(a)
	tick_params(direction="in")
end

plot(m::SearchDomain, f::EKF; alpha=0.1) = plot(m, f.mu, f.Sigma)
plot(m::SearchDomain, f::UKF; alpha=0.1) = plot(m, f.mu, f.Sigma)

# Plots mean and 95% confidence ellipse
function plot(m::SearchDomain, mu::LocTuple, Sigma::Matrix{Float64}; color="b")
	plot(m, [mu[1],mu[2]], Sigma, color=color)
end
function plot(m::SearchDomain, mu::Vector{Float64}, Sigma::Matrix{Float64}; color="b")
	a = [0,m.length,0,m.length]

	Sigma_half = sqrtm(Sigma)
	Sh11 = Sigma_half[1,1]
	Sh12 = Sigma_half[1,2]
	Sh21 = Sigma_half[2,1]
	Sh22 = Sigma_half[2,2]
	m1 = mu[1]
	m2 = mu[2]

	thetas = 0:.02:2pi
	num_theta = length(thetas)
	xvals = zeros(num_theta)
	yvals = zeros(num_theta)
	c = sqrt(-2.0*log(0.05))
	for i = 1:num_theta
		theta = thetas[i]
		s_theta = sin(theta)
		c_theta = cos(theta) 
		xvals[i] = c*(Sh11*s_theta + Sh12*c_theta) + m1
		yvals[i] = c*(Sh21*s_theta + Sh22*c_theta) + m2
	end
	plot(xvals, yvals, color)
	plot(m1, m2, "$(color)x", ms=10)

	labels()
	#axis("square")
	axis(a)
	tick_params(direction="in")
end

#function plot_ellipse()
#end


function plot(m::SearchDomain, f::PF; alpha=1.0)
	mark_size = 12
	a = [0,m.length,0,m.length]
	x = zeros(f.n)
	y = zeros(f.n)
	for i = 1:f.n
		x[i] = f.X[i][1]
		y[i] = f.X[i][2]
	end
	#scatter(x,y,c=f.W,cmap="Greys",vmin=0)
	scatter(x,y,c=f.W,cmap="Blues",vmin=0, alpha=0.2)
	xmean, ymean = centroid(f)
	plot(xmean, ymean, "gx", ms=10, mew=2)
	labels()
	axis("scaled")
	axis(a)
	tick_params(direction="in")
end

function plot_mean(mu::LocTuple; color="b")
	plot_mean(mu[1], mu[2], color=color)
end
function plot_mean(xmean::Float64, ymean::Float64; color="b")
	plot(xmean, ymean, "$(color)x", ms=10, mew=2)
end



# This is needed for plot_sim
#function plot_b(m::SearchDomain, b::Belief, x::Vehicle)
#	plot_theta(m)
#	hold(true)
#	plot_vehicles(X)
#	imshow(b', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#end

#"""
#`plot_eid(m::SearchDomain, X::VehicleSet)`
#
#Plots the eid using the Fisher information matrix.
#"""
#function plot_eid(m::SearchDomain, X::VehicleSet; contours=false)
#	eid = EID(m,m.b)
#	if contours
#		plot_contour(m, eid)
#	end
#	plot_theta(m)
#	plot_vehicles(X)
#	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#end

#"""
#`plot_mi(m::SearchDomain, X::VehicleSet)`
#
#Plots the mutual information.
#"""
#function plot_mi(m::SearchDomain, X::VehicleSet; contours=false)
#	mut_info = mutual_information(m)
#	plot_vehicles(X)
#	imshow(mut_info', interpolation="none", cmap="Greys", origin="lower")
#	labels()
#	if contours
#		plot_contour(m, mut_info)
#	end
#end


#"""
#`plot_sim(m::SearchDomain, s::Simulation)`
#
#Steps through a simulation.
#"""
#function plot_sim(m::SearchDomain, s::Simulation)
#	for t = 0:s.T
#		hold(false)
#		pause(1)
#		plot_b(m, s.belief_list[t+1], s.state_list[t+1])
#	end
#end


######################################################################
# Helper functions
# mec = markeredgecolor
# mfc = markerfacecolor
# lw = linewidth
######################################################################
# Plots locations of the vehicles
plot_vehicle(m::SearchDomain,x::Vehicle; color="b") = plot_vehicle(m,x.x,x.y,x.heading, color=color)
plot_vehicle(m::SearchDomain, p::Pose; color="b") = plot_vehicle(m, p[1], p[2], p[3], color=color)
function plot_vehicle(m::SearchDomain, x::Float64, y::Float64, h::Float64; color="b")
	mark_size = 10

	# Changed by LD for 01/30/2017 for rss submission
	#c = 0.01 * m.length  *sqrt(2)
	#c = 0.015 * m.length  *sqrt(2)
	#c = 0.012 * m.length  *sqrt(2)
	c = 0.013 * m.length  *sqrt(2)
	dx = c*sind(45 + h)
	dy = c*cosd(45 + h)

	# Plot rotors
	# LD change 01/30/2017 for rss submission
	#rotor_size = 5
	#rotor_size = 7.5
	rotor_size = 5.5
	rotor_size = 6.5
	theta1 = 45.0 + h
	theta2 = 135.0 + h
	theta3 = 225.0 +  h
	theta4 = 315.0 + h
	#cs = "$(color)o"
	cs = "wo"
	color = "k"
	plot(x+c*sind(theta1), y+c*cosd(theta1), cs, ms=rotor_size, mew=2.5, mfc="none", mec=color)
	plot(x+c*sind(theta2), y+c*cosd(theta2), cs, ms=rotor_size, mew=2.5, mfc="none", mec=color)
	plot(x+c*sind(theta3), y+c*cosd(theta3), cs, ms=rotor_size, mew=2.5, mfc="none", mec=color)
	plot(x+c*sind(theta4), y+c*cosd(theta4), cs, ms=rotor_size, mew=2.5, mfc="none", mec=color)

	# plot heading direction
	xline = [x, x+2*c*sind(h)]
	yline = [y, y+2*c*cosd(h)]
	# Changed by LD 01/30/2017 for rss submission
	#plot(xline, yline, color, mew=2)
	#plot(xline, yline, color, lw=2)
	#plot(xline, yline, color, lw=2.5)
	plot(xline, yline, color, lw=2.)

	# Plot frame
	plot(x, y, marker=(2,0,-theta1), ms=mark_size, mew=1, mec=color)
	plot(x, y, marker=(2,0,-theta2), ms=mark_size, mew=1, mec=color)

end

# Plots jammer location
function plot_theta(m::SearchDomain)
	# changed by LD 01/30/2017 for RSS submission
	#mark_size = 11
	#mark_size = 13
	mark_size = 11
	#mark_size = 13
	plot(m.theta[1], m.theta[2], "k^", markersize=mark_size, mew=2.5,markerfacecolor="none")
	#mark_size = 11
	#plot(m.theta[1], m.theta[2], "r^", markersize=mark_size, markerfacecolor="none", markeredgecolor="r", mew=2)
end

# Plots contours of some distribution `d` (a matrix).
function plot_contour(m::SearchDomain, d::Matrix{Float64})
	X,Y = meshgrid(linspace(0.,m.length,100), linspace(0.,m.length,100))
	contour(X, Y, d')
end

# Sets the appropriate plot labels
function labels()
	xlabel("East (m)", fontsize=12)
	ylabel("North (m)", fontsize=12)
end

######################################################################
# Old, deprecated, or unused
######################################################################
function plot_eid2(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	plot(x[1], x[2], "b*", markersize=mark_size)
	plot(theta[1], theta[2], "r^", markersize=mark_size)
	imshow(eid', interpolation="none", cmap="Greys", origin="lower")
	xlabel("x")
	ylabel("y")
end

function plot_eid3(m, b, x, theta)
	mark_size = 12
	eid = EID(m,theta[1],theta[2])
	#plot(x[1], m.num_cells - 1 - x[2], "b*", markersize=mark_size)
	#plot(theta[1], m.num_cells - 1 - theta[2], "r^", markersize=mark_size)
	#eid_plot = (eid')[end:-1:1, :]

	#imshow(eid_plot, interpolation="none", cmap="Greys")
	plot_contour(m, eid)
	xlabel("x")
	ylabel("y")
end

"""
`meshgrid(x, y)`

Returns X and Y.
"""
function meshgrid(x, y)
	nx = length(x)
	ny = length(y)

	X = zeros(ny, nx)
	Y = zeros(ny, nx)
	for xi = 1:nx
		for xj = 1:ny
			X[xj, xi] = x[xi]
			Y[xj, xi] = y[xj]
		end
	end
	return X, Y
end

######################################################################
# plotting multiple plots...
######################################################################
# TODO: make this just call the other version
function plot{TP<:AbstractFilter}(m::SearchDomain, farr::Vector{TP}, xarr::Vector{Vehicle}; show_mean::Bool=false, show_cov::Bool=false)
	for i = 1:length(farr)
		f = farr[i]
		x = xarr[i]
		plot(m, f, (x.x, x.y, x.heading); show_mean=show_mean, show_cov=show_cov)
	end
end

function plot{TP<:AbstractFilter}(m::SearchDomain, farr::Vector{TP}, parr::Vector{Pose}; show_mean::Bool=false, show_cov::Bool=false, alpha=1.0)
	num_vehicles = length(farr)
	cmaps = ["Blues", "Reds"]
	colors = ["b", "r"]
	if num_vehicles == 1
		cmaps = ["Greys"]
		colors = ["b"]
	elseif num_vehicles == 2
		cmaps = ["Blues", "Reds"]
		colors = ["b", "r"]
	elseif num_vehicles == 3
		cmaps = ["Blues", "Reds", "Greens"]
		colors = ["b", "r", "g"]
	elseif num_vehicles == 4
		cmaps = ["Blues", "Reds", "Greens", "Greys"]
		colors = ["b", "r", "g", "k"]
	end
	plot_theta(m)
	#hold(true)  # deprecated
	#cmaps = ["Blues", "Reds"]
	#colors = ["b", "r"]
	for (fi,f) in enumerate(farr)
		p = parr[fi]
		plot_vehicle(m,p, color=colors[fi])
		if show_mean
			plot_mean(centroid(f), color=colors[fi])
		end
		if show_cov
			plot(m, centroid(f), covariance(f), color=colors[fi])
		end
		plot(m, f, alpha=alpha, cmap=cmaps[fi])
	end
	return # so it doesn't spit out result of axis
end
