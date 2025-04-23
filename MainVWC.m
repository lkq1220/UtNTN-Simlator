clc;
clear;
close all;

axesFontSize = 14;
tic
%% LoRaWAN configurations
Gt=(10.^((2.15)/10));       %2.15 dBi 
Ptx = 10^(14/10)/1000;      % Transmit Power of LoRa 14 dBm
Frequency = 433e6;           % in Hz
MonteCarlo = 1e2;

% D_SNR=10.^([-6, -15, -20]./10);  %Demodulated SNR of LoRa SF12
D_SNR=10.^(-15./10);
Noise_Figure=6;
Band_Width=125e3; %LoRa BW
NoisePower=(10.^((-174+Noise_Figure+10*log10(Band_Width))./10))/1000;
R = 6378e3;

%%LR-FHSS configuration
D_SNR_FHSS = 10.^(4./10);
NoisePower_FHSS = (10.^((-174+Noise_Figure+10*log10(488))./10))/1000;

%% NB-IoT configurations
Ptx_NB = 10^(23/10)/1000;
Gt_NB=(10.^((1)/10));       %2.15 dBi
Frequency_NB = 450e6;           % in Hz
Noise_Figure_NB = 3;
Band_Width_NBUP = 15e3;

NoisePower_NB = (10.^((-174+Noise_Figure_NB+10*log10(Band_Width_NBUP))./10))/1000;
D_SNR_NB = 10.^(-11.8./10);
NB_receiver_sen = round(-174+Noise_Figure_NB+10*log10(Band_Width_NBUP)+D_SNR_NB);

%Soil loss
clay_input=16.86;
vwc_input=[0.05 0.15 0.25];
Depth=0.6;
for i = 1:length(vwc_input)
    [RealSoilDielectric, ImagSoilDielectric] = clc_die(clay_input, vwc_input(i), Frequency);
    Ls(i) = U2Aloss(RealSoilDielectric, ImagSoilDielectric,Depth,Frequency);
end

%Pipe loss
%//--------plasic Dielectric calculation
thick_pla = 0.05;
thick_asphalt = 0.1;
real_die_pla = 3;
real_die_asphalt = 7;
% imag_die_pla = 0.1;
mp_pla = 1;           %magnetic permeability
ec_pla = 0.00001;     %https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10122202
ec_asphalt = 0.005;
%----------//
Lp = Pipeloss(real_die_pla, mp_pla, ec_pla, thick_pla, Frequency);
L_asphalt = Pipeloss(real_die_asphalt, mp_pla, ec_asphalt, thick_asphalt, Frequency);
%Refraction loss air-plastic plastic-soil soil-air
MIU_0 = 4 * pi * 10^-7;
EPSILON_0 =  8.854187817 .* 10.0.^-12; 

z_asphalt = sqrt(MIU_0/(EPSILON_0*real_die_asphalt));
z_pla = sqrt(MIU_0/(EPSILON_0*real_die_pla));
z_soil = sqrt(MIU_0/(EPSILON_0*RealSoilDielectric));
z_air = sqrt(MIU_0/EPSILON_0);

r_ap = abs((z_air-z_pla)./(z_air+z_pla))^2;
r_ps = abs((z_soil-z_pla)./(z_soil+z_pla))^2;
r_st = abs((z_asphalt-z_soil)./(z_asphalt+z_soil))^2;
r_ta = abs((z_air-z_asphalt)./(z_air+z_asphalt)).^2;

Lt_ap=-10*log10(1-r_ap); %Refraction loss on air-plastic
Lt_ps=-10*log10(1-r_ps); %Refraction loss on plastic-soil
Lt_st=-10*log10(1-r_st);  %Refraction loss on soil-asphalt
Lt_ta=-10*log10(1-r_ta); %Refraction loss on asphalt-air

%Total path losses except for air loss
Total_loss_tmp = Lt_ap+Lp+Lt_ps+Ls+Lt_st+L_asphalt+Lt_ta;
%%%only soil%%%
% r_sa = abs((z_air-z_soil)./(z_air+z_soil)).^2;
% Lt_sa=-10*log10(1-r_sa); %Refraction loss on asphalt-air
% Total_loss_tmp = Ls+Lt_sa;

Total_loss_tmp_W = 10.^(Total_loss_tmp./10);

%UAV Configuration
UAV_height = 100;
Gr_U = (10.^((2)/10));
Elevation_Angles_steps = 10:1:90;
Distance_U = UAV_height./sin(pi/2.*(Elevation_Angles_steps/90));

%HAP configuration
Elevation_Angles_steps = 10:1:90;
Gr_H=(10.^((17)/10));       %HAP receiver antenna gain is 17 dBi ref->
Orbital_height_H = 20e3;
Distance_H = height2range(Orbital_height_H,1,Elevation_Angles_steps);

%Satellite configuration
Gr_S=(10.^((35)/10));       %Satellite receiver antenna gain is 35 dBi
Orbital_height_S = 550e3;     %Satellite height
Distance_S = height2range(Orbital_height_S,1,Elevation_Angles_steps);

index_list_U = [1 2 3 4 6 8 10 15 21 33 81];
index_list_H = [1 2 3 5 10 14 21 33 81];
index_list_S = [1 2 4 6 8 10 12 14 17 21 26 32 41 56 81];

%LORAWAN--Plot Results Under Different Depths 
for index = 1:1:length(vwc_input)
    index
    %% UAV air loss

    % [Rural_Loss_U, Urban_Loss_U, Distance_U] = UAVAirLossLow(UAV_height, MonteCarlo, Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W);
    [Rural_Loss_U, Urban_Loss_U, Rural_LadB_U, Urban_LadB_U] = UAVAirLossHigh(UAV_height, Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W(index), Distance_U, MonteCarlo);
    [Rural_Loss_U_NB, Urban_Loss_U_NB, Rural_LadB_U_NB, Urban_LadB_U_NB] = UAVAirLossHigh(UAV_height, Frequency_NB, Ptx_NB, Gr_U, Gt_NB, Total_loss_tmp_W(index), Distance_U, MonteCarlo);

    %% HAP air loss
    [Rural_Loss_H, Urban_Loss_H, Dense_Loss_H, Rural_LadB_H, Urban_LadB_H, Dense_LadB_H] = LEOHAPAirLoss(MonteCarlo, Frequency, Ptx, Gr_H, Gt, Total_loss_tmp_W(index), Distance_H, Elevation_Angles_steps);
    [Rural_Loss_H_NB, Urban_Loss_H_NB, Dense_Loss_H_NB, Rural_LadB_H_NB, Urban_LadB_H_NB, Dense_LadB_H_NB] = LEOHAPAirLoss(MonteCarlo, Frequency_NB, Ptx_NB, Gr_H, Gt_NB, Total_loss_tmp_W(index), Distance_H, Elevation_Angles_steps);

    %% Satellite air loss
    [Rural_Loss_S, Urban_Loss_S, Dense_Loss_S, Rural_LadB_S, Urban_LadB_S, Dense_LadB_S] = LEOHAPAirLoss(MonteCarlo, Frequency, Ptx, Gr_S, Gt, Total_loss_tmp_W(index), Distance_S, Elevation_Angles_steps);
    [Rural_Loss_S_NB, Urban_Loss_S_NB, Dense_Loss_S_NB, Rural_LadB_S_NB, Urban_LadB_S_NB, Dense_LadB_S_NB] = LEOHAPAirLoss(MonteCarlo, Frequency_NB, Ptx_NB, Gr_S, Gt_NB, Total_loss_tmp_W(index), Distance_S, Elevation_Angles_steps);

    for k=1:1:length(index_list_U)
        Rural_PSNR_U_NB(index, k)=sum((Rural_Loss_U_NB(index_list_U(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
        Urban_PSNR_U_NB(index, k)=sum((Urban_Loss_U_NB(index_list_U(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
    end

    for k=1:1:length(index_list_H)
        Rural_PSNR_H_NB(index, k)=sum((Rural_Loss_H_NB(index_list_H(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
        Urban_PSNR_H_NB(index, k)=sum((Urban_Loss_H_NB(index_list_H(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
        Dense_PSNR_H_NB(index, k)=sum((Dense_Loss_H_NB(index_list_H(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
    end

    for k=1:1:length(index_list_S)
        Rural_PSNR_S_NB(index, k)=sum((Rural_Loss_S_NB(index_list_S(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
        Urban_PSNR_S_NB(index, k)=sum((Urban_Loss_S_NB(index_list_S(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
        Dense_PSNR_S_NB(index, k)=sum((Dense_Loss_S_NB(index_list_S(k),:)./NoisePower_NB)>=D_SNR_NB)/MonteCarlo;
    end

    for k=1:1:length(index_list_U)
        Rural_PSNR_U_FHSS(index, k)=sum((Rural_Loss_U(index_list_U(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
        Urban_PSNR_U_FHSS(index, k)=sum((Urban_Loss_U(index_list_U(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
    end

    for k=1:1:length(index_list_H)
        Rural_PSNR_H_FHSS(index, k)=sum((Rural_Loss_H(index_list_H(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
        Urban_PSNR_H_FHSS(index, k)=sum((Urban_Loss_H(index_list_H(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
        Dense_PSNR_H_FHSS(index, k)=sum((Dense_Loss_H(index_list_H(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
    end

    for k=1:1:length(index_list_S)
        Rural_PSNR_S_FHSS(index, k)=sum((Rural_Loss_S(index_list_S(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
        Urban_PSNR_S_FHSS(index, k)=sum((Urban_Loss_S(index_list_S(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
        Dense_PSNR_S_FHSS(index, k)=sum((Dense_Loss_S(index_list_S(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
    end

    for k=1:1:length(index_list_U)
        Rural_PSNR_U_SF(index, k)=sum((Rural_Loss_U(index_list_U(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
        Urban_PSNR_U_SF(index, k)=sum((Urban_Loss_U(index_list_U(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
    end

    for k=1:1:length(index_list_H)
        Rural_PSNR_H_SF(index, k)=sum((Rural_Loss_H(index_list_H(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
        Urban_PSNR_H_SF(index, k)=sum((Urban_Loss_H(index_list_H(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
        Dense_PSNR_H_SF(index, k)=sum((Dense_Loss_H(index_list_H(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
    end

    for k=1:1:length(index_list_S)
        Rural_PSNR_S_SF(index, k)=sum((Rural_Loss_S(index_list_S(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
        Urban_PSNR_S_SF(index, k)=sum((Urban_Loss_S(index_list_S(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
        Dense_PSNR_S_SF(index, k)=sum((Dense_Loss_S(index_list_S(k),:)./NoisePower)>=D_SNR)/MonteCarlo;
    end

end
ToA = 370.688; %ToA for SF7 SF10 SF12 
R_Time = 600;                    %Report period
Dutycycle=ToA./(1000*R_Time);
P_SIR=zeros(length(D_SNR),length(Distance_U));

Node_number = 625;
Channels = 1;
gamma_db = 6; % dB - SIR Threshold
gamma_th = 10^(gamma_db/10); % linear - SIR Threshold
for count=1:length(vwc_input)
    for k_U=1:1:length(index_list_U)
        [count k_U]
        discarded_U_Rural = 0;
        discarded_U_Urban = 0;
        for m=1:MonteCarlo
            simultaneous = poissrnd(2*Dutycycle*Node_number./Channels,1);
                if simultaneous > 0                                  % if there is any simultaneous transmission (if collision)
                    %% Generating location of interfering signals
                    clear Coordinates                    
                    clear rho
                    clear Theta
                    Coordinates=zeros(simultaneous,2);   
                    rho = sqrt(rand(simultaneous,1).*(sqrt(max(Distance_U)^2 - UAV_height^2)^2));       
                    Theta = rand(simultaneous,1)*2*pi;                      
                    Coordinates(:,1) = cos(Theta).*rho;
                    Coordinates(:,2) = sin(Theta).*rho;
                    Location_Nodes_Int = sqrt(Coordinates(:,1).^2 + Coordinates(:,2).^2)';
                    dPropogation=zeros(1,simultaneous); 

                    % Distance from interfering nodes to NTN
                    for track=1:length(Location_Nodes_Int)
                       dPropogation = sqrt(UAV_height^2 + Location_Nodes_Int(track).^2); 
                       E_AngPro_U = rad2deg(asin(UAV_height/dPropogation));
                       Rural_LadB_U_I(track) = Rural_LadB_U(round((E_AngPro_U)-9));
                       Urban_LadB_U_I(track) = Urban_LadB_U(round((E_AngPro_U)-9));
                       % [Rural_Loss_U_I, Urban_Loss_U_I, Rural_LadB_U_I(track), Urban_LadB_U_I(track)] = UAVAirLossHigh(UAV_height, Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W, dPropogation(track));
                    end

                    pr_h_g_I_U_Rural = sum((Ptx*Gr_U*Gt)./(10.^(Rural_LadB_U_I./10))./Total_loss_tmp_W(count));
                    pr_h_g_I_U_Urban = sum((Ptx*Gr_U*Gt)./(10.^(Urban_LadB_U_I./10))./Total_loss_tmp_W(count));

                    pr_h_g_D_U_Rural = Ptx*Gr_U*Gt./(10.^(Rural_LadB_U(index_list_U(k_U))./10))./Total_loss_tmp_W(count);
                    pr_h_g_D_U_Urban = Ptx*Gr_U*Gt./(10.^(Urban_LadB_U(index_list_U(k_U))./10))./Total_loss_tmp_W(count);

                    if  pr_h_g_D_U_Rural  < (pr_h_g_I_U_Rural*gamma_th)
                      discarded_U_Rural = discarded_U_Rural + 1; %% Destructive collision
                    end

                    if  pr_h_g_D_U_Urban  < (pr_h_g_I_U_Urban*gamma_th)
                      discarded_U_Urban = discarded_U_Urban + 1; %% Destructive collision
                    end

                 end
         end
         P_SIR_U_Rural(count, k_U) = 1- (discarded_U_Rural/MonteCarlo); %% Success in presence of interfering nodes
         P_SIR_U_Urban(count, k_U) = 1- (discarded_U_Urban/MonteCarlo); %% Success in presence of interfering nodes
    end

    for k_H=1:1:length(index_list_H)
        [count k_H]
        discarded_H_Rural = 0;
        discarded_H_Urban = 0;
        discarded_H_Dense = 0;

        for m=1:MonteCarlo
            simultaneous = poissrnd(2*Dutycycle*Node_number./Channels,1);
                if simultaneous > 0                                  % if there is any simultaneous transmission (if collision)
                    %% Generating location of interfering signals
                    clear Coordinates_H                    
                    clear rho_H 
                    clear Theta_H 
                    Coordinates_H = zeros(simultaneous,2);   
                    rho_H = sqrt(rand(simultaneous,1).*(sqrt(max(Distance_H)^2 - Orbital_height_H^2)^2));       
                    Theta_H = rand(simultaneous,1)*2*pi;                      
                    Coordinates_H(:,1) = cos(Theta_H).*rho_H;
                    Coordinates_H(:,2) = sin(Theta_H).*rho_H;
                    Location_Nodes_Int_H = sqrt(Coordinates_H(:,1).^2 + Coordinates_H(:,2).^2)';

                    % Distance from interfering nodes to HAP
                    for track=1:length(Location_Nodes_Int_H)
                       dPropogation_H = sqrt(Orbital_height_H^2 + Location_Nodes_Int_H(track).^2); 
                       E_AngPro_H = height2el(Orbital_height_H,1,dPropogation_H);
                       Rural_LadB_H_I(track) = Rural_LadB_H(round(E_AngPro_H)-9);
                       Urban_LadB_H_I(track) = Urban_LadB_H(round(E_AngPro_H)-9);
                       Dense_LadB_H_I(track) = Dense_LadB_H(round(E_AngPro_H)-9);
                    end
                    pr_h_g_I_H_Rural = sum((Ptx*Gr_H*Gt)./(10.^(Rural_LadB_H_I./10))./Total_loss_tmp_W(count));    
                    pr_h_g_D_H_Rural = Ptx*Gr_H*Gt./(10.^(Rural_LadB_H(index_list_H(k_H))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_H_Rural  < (pr_h_g_I_H_Rural*gamma_th)
                        discarded_H_Rural = discarded_H_Rural + 1; %% Destructive collision
                    end
                    
                    pr_h_g_I_H_Urban = sum((Ptx*Gr_H*Gt)./(10.^(Urban_LadB_H_I./10))./Total_loss_tmp_W(count));    
                    pr_h_g_D_H_Urban = Ptx*Gr_H*Gt./(10.^(Urban_LadB_H(index_list_H(k_H))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_H_Urban  < (pr_h_g_I_H_Urban*gamma_th)
                        discarded_H_Urban = discarded_H_Urban + 1; %% Destructive collision
                    end
                    
                    pr_h_g_I_H_Dense = sum((Ptx*Gr_H*Gt)./(10.^(Dense_LadB_H_I./10))./Total_loss_tmp_W(count));    
                    pr_h_g_D_H_Dense = Ptx*Gr_H*Gt./(10.^(Dense_LadB_H(index_list_H(k_H))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_H_Dense  < (pr_h_g_I_H_Dense*gamma_th)
                        discarded_H_Dense = discarded_H_Dense + 1; %% Destructive collision
                    end

                 end
         end
         P_SIR_H_Rural(count, k_H) = 1-(discarded_H_Rural/MonteCarlo); %% Success in presence of interfering nodes
         P_SIR_H_Urban(count, k_H) = 1-(discarded_H_Urban/MonteCarlo);
         P_SIR_H_Dense(count, k_H) = 1-(discarded_H_Dense/MonteCarlo);
    end

    for k_S=1:1:length(index_list_S)
        [count k_S]
        discarded_S_Rural = 0;
        discarded_S_Urban = 0;
        discarded_S_Dense = 0;
    
        for m=1:MonteCarlo
            simultaneous = poissrnd(2*Dutycycle*Node_number./Channels,1);
                if simultaneous > 0                                  % if there is any simultaneous transmission (if collision)
                    %% Generating location of interfering signals
                    clear Coordinates_S                    
                    clear rho_S 
                    clear Theta_S 
                    Coordinates_S = zeros(simultaneous,2);   
                    rho_S = sqrt(rand(simultaneous,1).*(sqrt(max(Distance_S)^2 - Orbital_height_S^2)^2));       
                    Theta_S = rand(simultaneous,1)*2*pi;                      
                    Coordinates_S(:,1) = cos(Theta_S).*rho_S;
                    Coordinates_S(:,2) = sin(Theta_S).*rho_S;
                    Location_Nodes_Int_S = sqrt(Coordinates_S(:,1).^2 + Coordinates_S(:,2).^2)';

                    % Distance from interfering nodes to HAP
                    for track=1:length(Location_Nodes_Int_S)
                       dPropogation_S = sqrt(Orbital_height_S^2 + Location_Nodes_Int_S(track).^2); 
                       E_AngPro_S = height2el(Orbital_height_S,1,dPropogation_S);
                       Rural_LadB_S_I(track) = Rural_LadB_S(round(E_AngPro_S)-9);
                       Urban_LadB_S_I(track) = Urban_LadB_S(round(E_AngPro_S)-9);
                       Dense_LadB_S_I(track) = Dense_LadB_S(round(E_AngPro_S)-9);
                    end

                    pr_h_g_I_S_Rural = sum((Ptx*Gr_S*Gt)./(10.^(Rural_LadB_S_I./10))./Total_loss_tmp_W(count));             
                    pr_h_g_D_S_Rural = Ptx*Gr_S*Gt./(10.^(Rural_LadB_S(index_list_S(k_S))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_S_Rural  < (pr_h_g_I_S_Rural*gamma_th)
                        discarded_S_Rural = discarded_S_Rural + 1; %% Destructive collision
                    end

                    pr_h_g_I_S_Urban = sum((Ptx*Gr_S*Gt)./(10.^(Urban_LadB_S_I./10))./Total_loss_tmp_W(count));             
                    pr_h_g_D_S_Urban = Ptx*Gr_S*Gt./(10.^(Urban_LadB_S(index_list_S(k_S))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_S_Urban  < (pr_h_g_I_S_Urban*gamma_th)
                        discarded_S_Urban = discarded_S_Urban + 1; %% Destructive collision
                    end

                    pr_h_g_I_S_Dense = sum((Ptx*Gr_S*Gt)./(10.^(Dense_LadB_S_I./10))./Total_loss_tmp_W(count));             
                    pr_h_g_D_S_Dense = Ptx*Gr_S*Gt./(10.^(Dense_LadB_S(index_list_S(k_S))./10))./Total_loss_tmp_W(count);
                    if  pr_h_g_D_S_Dense  < (pr_h_g_I_S_Dense*gamma_th)
                        discarded_S_Dense = discarded_S_Dense + 1; %% Destructive collision
                    end

                 end
         end
         P_SIR_S_Rural(count, k_S) = 1-(discarded_S_Rural/MonteCarlo); %% Success in presence of interfering nodes
         P_SIR_S_Urban(count, k_S) = 1-(discarded_S_Urban/MonteCarlo);
         P_SIR_S_Dense(count, k_S) = 1-(discarded_S_Dense/MonteCarlo);
    end
end

Ps_SF10_U_2 =  Rural_PSNR_U_SF(1,:).*P_SIR_U_Rural(1,:);
Ps_SF10_U_6 =  Rural_PSNR_U_SF(2,:).*P_SIR_U_Rural(2,:);
Ps_SF10_U_10 = Rural_PSNR_U_SF(3,:).*P_SIR_U_Rural(3,:);

Ps_FHSS_U_2 =  importdata("FHSS_U_Res.mat")./1e4.*Rural_PSNR_U_FHSS(1,:);
Ps_FHSS_U_6 =  importdata("FHSS_U_Res.mat")./1e4.*Rural_PSNR_U_FHSS(2,:);
Ps_FHSS_U_10 =  importdata("FHSS_U_Res.mat")./1e4.*Rural_PSNR_U_FHSS(3,:);

Ps_SF10_H_2 =  Rural_PSNR_H_SF(1,:).*P_SIR_H_Rural(1,:);
Ps_SF10_H_6 =  Rural_PSNR_H_SF(2,:).*P_SIR_H_Rural(2,:);
Ps_SF10_H_10 =  Rural_PSNR_H_SF(3,:).*P_SIR_H_Rural(3,:);

Ps_FHSS_H_2 =  importdata("FHSS_H_Res.mat")./1e4.*Rural_PSNR_H_FHSS(1,:);
Ps_FHSS_H_6 =  importdata("FHSS_H_Res.mat")./1e4.*Rural_PSNR_H_FHSS(2,:);
Ps_FHSS_H_10 =  importdata("FHSS_H_Res.mat")./1e4.*Rural_PSNR_H_FHSS(3,:);

Ps_SF10_S_2 =  Rural_PSNR_S_SF(1,:).*P_SIR_S_Rural(1,:);
Ps_SF10_S_6 =  Rural_PSNR_S_SF(2,:).*P_SIR_S_Rural(2,:);
Ps_SF10_S_10 =  Rural_PSNR_S_SF(3,:).*P_SIR_S_Rural(3,:);

Ps_FHSS_S_2 =  importdata("FHSS_S_Res.mat")./1e4.*Rural_PSNR_S_FHSS(1,:);
Ps_FHSS_S_6 =  importdata("FHSS_S_Res.mat")./1e4.*Rural_PSNR_S_FHSS(2,:);
Ps_FHSS_S_10 =  importdata("FHSS_S_Res.mat")./1e4.*Rural_PSNR_S_FHSS(3,:);


figure
set(gcf, 'Units', 'centimeters'); 
afFigurePosition = [2 7 16 12]; 
set(gcf, 'Position', afFigurePosition,'PaperSize',[18 8],'PaperPositionMode','auto'); 
h(1) = plot(Distance_U(index_list_U),Ps_SF10_U_2,'-o','SeriesIndex',1,'LineWidth',2.5);
hold on
h(2) = plot(Distance_U(index_list_U),Ps_SF10_U_6,'-^','SeriesIndex',2,'LineWidth',2.5);
hold on
h(3) = plot(Distance_U(index_list_U),Ps_SF10_U_10,'-d','SeriesIndex',3,'LineWidth',2.5);
hold on
h(4) = plot(Distance_U(index_list_U),Ps_FHSS_U_2,':o','SeriesIndex',4,'LineWidth',2.5);
hold on
h(5) = plot(Distance_U(index_list_U),Ps_FHSS_U_6,':^','SeriesIndex',5,'LineWidth',2.5);
hold on
h(6) = plot(Distance_U(index_list_U),Ps_FHSS_U_10,':d','SeriesIndex',6,'LineWidth',2.5);
hold on
xlabel('Distance from devices to gateway (m)','Interpreter','Latex','FontSize', axesFontSize)
ylabel('Probability ($P_{s}^{uav}$)','Interpreter','Latex','FontSize', axesFontSize)
%legend('$d_3$=0.2 m (LoRa SF10)','$d_3$=0.6 m (LoRa SF10)','$d_u$=1.0 m (LoRa SF10)','$d_u$=0.2 m (LR-FHSS)','$d_u$=0.6 m (LR-FHSS)','$d_u$=1.0 m (LR-FHSS)','Location','best','Interpreter','Latex','FontSize', axesFontSize);
grid on
axis([Distance_U(end) 600 0 1]);
set(gca,'FontSize',axesFontSize);
set(gca, 'LooseInset', [0,0,0,0]);

LoRaMod=legend(h(1:3),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(LoRaMod,'LoRa (SF10)');
LoRaMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');
hold on

FHSSMod=legend(LoRaMod_ax,h(4:6),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(FHSSMod,'LR-FHSS')
FHSSMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');

figure
set(gcf, 'Units', 'centimeters'); 
afFigurePosition = [2 7 16 12]; 
set(gcf, 'Position', afFigurePosition,'PaperSize',[18 8],'PaperPositionMode','auto'); 
h(1) = plot(Distance_H(index_list_H)/1e3,Ps_SF10_H_2,'-o','SeriesIndex',1,'LineWidth',2.5);
hold on
h(2) = plot(Distance_H(index_list_H)/1e3,Ps_SF10_H_6,'-^','SeriesIndex',2,'LineWidth',2.5);
hold on
h(3) = plot(Distance_H(index_list_H)/1e3,Ps_SF10_H_10,'-d','SeriesIndex',3,'LineWidth',2.5);
hold on

h(4) = plot(Distance_H(index_list_H)/1e3,Ps_FHSS_H_2,':o','SeriesIndex',4,'LineWidth',2.5);
hold on
h(5) = plot(Distance_H(index_list_H)/1e3,Ps_FHSS_H_6,':^','SeriesIndex',5,'LineWidth',2.5);
hold on
h(6) = plot(Distance_H(index_list_H)/1e3,Ps_FHSS_H_10,':d','SeriesIndex',6,'LineWidth',2.5);
hold on
xlabel('Distance from devices to gateway (km)','Interpreter','Latex','FontSize', axesFontSize)
ylabel('Probability ($P_{s}^{hap}$)','Interpreter','Latex','FontSize', axesFontSize)
% legend('$d_u$=0.2 m (LoRa SF10)','$d_u$=0.6 m (LoRa SF10)','$d_u$=1.0 m (LoRa SF10)','$d_u$=0.2 m (LR-FHSS)','$d_u$=0.6 m (LR-FHSS)','$d_u$=1.0 m (LR-FHSS)','Location','best','Interpreter','Latex','FontSize', axesFontSize, 'NumColumnsMode','manual','NumColumns',2);
grid on
axis([Distance_H(end)/1e3 120 0 1]);
set(gca,'FontSize',axesFontSize);
set(gca, 'LooseInset', [0,0,0,0]);

LoRaMod=legend(h(1:3),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(LoRaMod,'LoRa (SF10)');
LoRaMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');
hold on

FHSSMod=legend(LoRaMod_ax,h(4:6),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(FHSSMod,'LR-FHSS')
FHSSMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');

figure
set(gcf, 'Units', 'centimeters'); 
afFigurePosition = [2 7 16 12]; 
set(gcf, 'Position', afFigurePosition,'PaperSize',[18 8],'PaperPositionMode','auto'); 
h(1) = plot(Distance_S(index_list_S)/1e3,Ps_SF10_S_2,'-o','SeriesIndex',1,'LineWidth',2.5);
hold on
h(2) = plot(Distance_S(index_list_S)/1e3,Ps_SF10_S_6,'-^','SeriesIndex',2,'LineWidth',2.5);
hold on
h(3) = plot(Distance_S(index_list_S)/1e3,Ps_SF10_S_10,'-d','SeriesIndex',3,'LineWidth',2.5);
hold on
h(4) = plot(Distance_S(index_list_S)/1e3,Ps_FHSS_S_2,':o','SeriesIndex',4,'LineWidth',2.5);
hold on
h(5) = plot(Distance_S(index_list_S)/1e3,Ps_FHSS_S_6,':^','SeriesIndex',5,'LineWidth',2.5);
hold on
h(6) = plot(Distance_S(index_list_S)/1e3,Ps_FHSS_S_10,':d','SeriesIndex',6,'LineWidth',2.5);
hold on
xlabel('Distance from devices to gateway (km)','Interpreter','Latex','FontSize', axesFontSize)
ylabel('Probability ($P_{s}^{leo}$)','Interpreter','Latex','FontSize', axesFontSize)
% legend('$d_u$=0.2 m (LoRa SF10)','$d_u$=0.6 m (LoRa SF10)','$d_u$=1.0 m (LoRa SF10)','$d_u$=0.2 m (LR-FHSS)','$d_u$=0.6 m (LR-FHSS)','$d_u$=1.0 m (LR-FHSS)','Location','best','Interpreter','Latex','FontSize', axesFontSize, 'NumColumnsMode','manual','NumColumns',2);
grid on
axis([5e2 2e3 0 1]);
set(gca,'FontSize',axesFontSize);
set(gca, 'LooseInset', [0,0,0,0]);
LoRaMod=legend(h(1:3),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(LoRaMod,'LoRa (SF10)');
LoRaMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');
hold on

FHSSMod=legend(LoRaMod_ax,h(4:6),{'$m_v$=5 \%','$m_v$=15 \%', '$m_v$=25 \%'},'Location','best','NumColumns',1, 'Interpreter', 'Latex','FontSize',14);
title(FHSSMod,'LR-FHSS')
FHSSMod_ax=axes('Position',get(gca,'Position'),'Visible','Off');
% save LoRaWANResVWC.mat;

toc