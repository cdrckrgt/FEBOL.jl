export kl_divergence

# P is truth, Q is approximation
function kl_divergence(P, Q)

    kld = 0.0

    p_sum = sum(P)
    q_sum = sum(Q)

    for i = 1:length(P)
        if P[i] > 0.0
            pn = P[i] / p_sum
            qn = Q[i] / q_sum

            kld += pn * log(pn / qn)
        end
    end
    return kld
end
