function [Lu] = U2Aloss(realSoilDielectric, imagSoilDielectric, depth, Freq_Band)
    m_realSoilDielectric = realSoilDielectric;
    m_imagSoilDielectric = imagSoilDielectric;
    m_frequecny = Freq_Band;
    EPSILON_0 =  8.854187817 * 10.0^-12;
    MIU_0 = 4 * pi * 10^-7;
    omega = 2 * pi * m_frequecny;
    magEle = (MIU_0 * m_realSoilDielectric * EPSILON_0 / 2.0);
    imagReal = sqrt(1 + (m_imagSoilDielectric / m_realSoilDielectric)^2);
    m_Alpha = omega * sqrt(magEle * (imagReal - 1));
    m_Beta =  omega * sqrt(magEle * (imagReal + 1));
    theta_i =asin(1.0./sqrt(m_realSoilDielectric));
    r1 = abs(depth) ./ cos(theta_i);
    % Lu = 6.4 + 20.*log10(r1) + 20.*log10(m_Beta) + 8.69.*m_Alpha.*r1;
    Lu = 8.69.*m_Alpha.*r1 +20.*log10(m_Beta);
    20.*log10(m_Beta)
end

