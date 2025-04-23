function [Loss_res] = Pipeloss(realSoilDielectric, mp, ec, depth, Freq_Band)
    EPSILON_0 =  8.854187817 * 10.0^-12;
    MIU_0 = 4 * pi * 10^-7;
    omega = 2 * pi * Freq_Band;

    m_Alpha = ec/2*sqrt(MIU_0*mp/(EPSILON_0*realSoilDielectric));
    m_Beta =  omega * sqrt(MIU_0*mp*(EPSILON_0*realSoilDielectric));
    % shadow_effect_dB = 6.4 + 20.*log10(depth)+20.*log10(m_Beta);
    Loss_res =8.69.*m_Alpha.*depth;
    % Loss_res = 6.4 + 20.*log10(depth) + 20.*log10(m_Beta) + 8.69.*m_Alpha.*depth;
end