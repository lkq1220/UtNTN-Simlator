%function [realSoilDielectric, imagSoilDielectric] = clc_die(sand, clay, ro_b,ro_s, vwc_input, frequency)
function [realSoilDielectric, imagSoilDielectric] = clc_die(clay_input, vwc_input, frequency)
      EPSILON_0 =  8.854187817 .* 10.0.^-12;
      PI = 3.14159265358979323846;

 %计算介电常数，通过MBSDM模型
    clay = clay_input;
    vwc = vwc_input;
    
    eps_0b = 79.8 - 85.4 .* 0.01 .* clay + 32.7 .* (10.^-4) .* clay .* clay;
    eps_0u = 100;
    m_vt = 0.02863 + 0.30673 .* 0.01 .* clay;
    tau_u = 8.5 .* 10.^-12;
    tau_b = 1.062 .* 10.^-11+ 3.45 .* 0.01 .* clay.* (10.^-12);
    sigma_b = 0.3112 + 0.467.*0.01.*clay;
    sigma_u = 0.3631 + 1.217 .* 0.01 .* clay;
    epsilon_inf = 4.9;
    
    k_d = 0.03952-0.04038 .* 0.01 .* clay;
    n_d = 1.634 - 0.539 .* 0.01 .* clay + 0.2748 .* 10.^-4 .* clay .* clay;
    
    reps_b = epsilon_inf + (eps_0b - epsilon_inf)./(1 + (2 .* PI .* frequency .* tau_b).*(2 .* PI .* frequency .* tau_b));
    reps_u = epsilon_inf + (eps_0u - epsilon_inf)./(1 + (2 .* PI .* frequency .* tau_u).*(2 .* PI .* frequency .* tau_u));
    
    imeps_b = (eps_0b - epsilon_inf)./(1 + (2 .* PI .* frequency .* tau_b).^2.0) .* (2 .* PI .* frequency .* tau_b) + sigma_b./(2 .* PI .* EPSILON_0 .* frequency);
    imeps_u = (eps_0u - epsilon_inf)./(1 + (2 .* PI .* frequency .* tau_u).^2.0) .* (2 .* PI .* frequency .* tau_u) + sigma_u./(2 .* PI .* EPSILON_0 .* frequency);

    n_b = sqrt(sqrt(reps_b.^2 + imeps_b.^2)+ reps_b) ./ sqrt(2);
    n_u = sqrt(sqrt(reps_u.^2 + imeps_u.^2)+ reps_u) ./ sqrt(2);

    k_b = sqrt(sqrt(reps_b.^2 + imeps_b.^2)- reps_b) ./ sqrt(2);
    k_u = sqrt(sqrt(reps_u.^2 + imeps_u.^2)- reps_u) ./ sqrt(2);
    
    if vwc>m_vt
        n_m = n_d + (n_b - 1) .* m_vt + (n_u - 1) .* (vwc - m_vt);
        k_m = k_d + k_b .* m_vt + k_u .* (vwc - m_vt);
    else
        n_m = n_d + (n_b - 1) .* vwc;
        k_m = k_d +k_b .* vwc;
    end
   realSoilDielectric = n_m .* n_m - k_m .* k_m;
   imagSoilDielectric = 2 .* n_m .* k_m;
end

