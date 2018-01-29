using FEBOL
using Base.Test

# SearchDomain tests
m = SearchDomain(400, 170, 210)
@test m.theta == (170.0, 210.0)
m = SearchDomain(400)
theta!(m)
theta!(m, 32, 40)
