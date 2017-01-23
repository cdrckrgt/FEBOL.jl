export SimUnit
export MaxNormThreshold, is_complete

abstract TerminationCondition

type EntropyThreshold <: TerminationCondition
	value::Float64
end

type MaxNormThreshold <: TerminationCondition
	value::Float64
end


type SimUnit
	x::Vehicle
	f::AbstractFilter
	p::Policy
end

# determines if a simulation has reached its 
function is_complete(f::AbstractFilter)
end
function is_complete(f::DF, mnt::MaxNormThreshold)
	ret_val = false
	if maximum(f.b) > mnt.value
		ret_val = true
	end
	return ret_val
end

function update!(su::SimUnit, o::Float64)
	update!(su.f, su.x, o)
end
function action(m::SearchDomain, su::SimUnit, o::Float64)
	return action(m, su.x, o, su.f, su.p)
end

function batchsim(m::SearchDomain, uav_array::Vector{SimUnit}, num_sims::Int, tc::TerminationCondition)
	costs = zeros(num_sims, length(uav_array))
	for sim_ind = 1:num_sims
		theta!(m)
		for (uav_ind, uav) in enumerate(uav_array)

			# reset the filter and vehicle
			reset!(uav.f)
			reset!(m, uav.x)

			# starts at -1 because after first round, should be zero
			temp_cost = -1.0
			while !is_complete(uav.f, tc)
				# observe, update filter, act
				o = observe(m, uav.x)
				update!(uav, o)
				a = action(m, uav, o)
				act!(m, uav.x, a)
				temp_cost += 1.0
			end

			costs[sim_ind, uav_ind] = temp_cost
		end
		println("Simulation ", sim_ind, " complete.")
	end
	return costs
end
