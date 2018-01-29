using FEBOL
using Base.Test

m = SearchDomain(400, 170, 210)
@test m.theta = (170.0, 210.0)
