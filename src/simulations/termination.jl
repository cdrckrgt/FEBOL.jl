######################################################################
# termination condition stuff
#
# TODO: I probably don't need to pass in step_count, the step
#  threshold could simply keep track of this as a field
#
# Although maybe it is useful to have it passed in?
######################################################################
export StepThreshold, MaxNormThreshold
export is_complete

abstract TerminationCondition

# determines if a simulation has reached its 
function is_complete(f::AbstractFilter, ::TerminationCondition, ::Int)
	error("Termination condition not implemented for this filter or termination condition")
end


type StepThreshold <: TerminationCondition
	value::Int
end
function is_complete(f::DF, st::StepThreshold, step_count::Int)
	ret_val = false
	if step_count >= st.value
		ret_val = true
	end
	return ret_val
end


type MaxNormThreshold <: TerminationCondition
	value::Float64
end
function is_complete(f::DF, mnt::MaxNormThreshold, ::Int)
	ret_val = false
	if maximum(f.b) > mnt.value
		ret_val = true
	end
	return ret_val
end


type EntropyThreshold <: TerminationCondition
	value::Float64
end
