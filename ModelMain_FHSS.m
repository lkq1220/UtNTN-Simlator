clc
clear 
close all

%% Packet and transmissions parameters
tic
Payload = 10;           % Message payload
Header_N_DR8 = 3;       % Header replicas
Code_Rate = 1/3;
Header_duration = 0.233; %233 ms long headers
F_duration = 0.1024;       %50 ms payload data fragments
Header_ToA_DR8 = Header_N_DR8*Header_duration;
%Nodes = [50 60 70 80 90 100].*1e3;
Nodes = [10e3 50e3 100e3];
Simulation_T = 3600; % 1 hour duration
pkct_p_h = 4;      % Packets per hour per end-device
OBW_channels=280;  % No. of OBW channels
MonteCarlo = 1e2;    % No. of Iterations
M = 2;             % M = 2 for DR8, M=4 for DR9
D_SNR_FHSS = 10.^(4./10);
Noise_Figure=6;
NoisePower_FHSS = (10.^((-174+Noise_Figure+10*log10(488))./10))/1000;


Gt=(10.^((2.15)/10));       %2.15 dBi 
Ptx = 10^(14/10)/1000;      % Transmit Power of LoRa 14 dBm
Frequency = 433e6;           % in Hz
%Soil loss
clay_input=16.86;
vwc_input=0.1119;
Depth=0.6;

[RealSoilDielectric, ImagSoilDielectric] = clc_die(clay_input, vwc_input, Frequency);
Ls = U2Aloss(RealSoilDielectric, ImagSoilDielectric,Depth,Frequency);

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

gamma_db = 6; % dB - SIR Threshold
gamma_th = 10^(gamma_db/10); % linear - SIR Threshold


%% Time on air
[ToA_DR8,ToA_DR8_WH] = ToA_Packets_DR8(Payload,Header_ToA_DR8,M); 
% ToA_DR8 -> including headers duration
% ToA_DR8_WH -> without headers duration
Transceiver_wait = 6.472/1000; %Tw      
ToA_DR8(1)=ToA_DR8(1) + Transceiver_wait;  % Total on-air time

%% Number of fragments
fragment_duration = 102.4/1000; %50 ms

fragment_50_ms = floor(ToA_DR8_WH(1)/fragment_duration);  %No. of payload data fragments
% The last fragment may not be equal to 50ms. We need to calculate it.
Last_fragment_duration = ((ToA_DR8_WH(1)/fragment_duration) - fragment_50_ms)*fragment_duration;
fragment_PHY_length  = fragment_50_ms + 1;
fragment_length = Header_N_DR8 + length(Transceiver_wait) + fragment_PHY_length;

%pack_tx_segments = Header_N_DR8 + ceil((0.102*ceil((Payload+2)/4))/F_duration) +length(Transceiver_wait);

%% Simulator
transmission_sim=zeros(MonteCarlo,fragment_length);

%UAV Distance 
UAV_height = 100;
Gr_U = (10.^((2)/10));
Elevation_Angles_steps = 10:1:90;
Distance_U = UAV_height./sin(pi/2.*(Elevation_Angles_steps/90));

%HAP Distance
Elevation_Angles_steps = 10:1:90;
Gr_H=(10.^((17)/10));       %HAP receiver antenna gain is 17 dBi ref->
Orbital_height_H = 20e3;
Distance_H = height2range(Orbital_height_H,1,Elevation_Angles_steps);

%Satellite Distance
Gr_S=(10.^((35)/10));       %Satellite receiver antenna gain is 35 dBi
Orbital_height_S = 550e3;     %Satellite height
Distance_S = height2range(Orbital_height_S,1,Elevation_Angles_steps);
[Rural_Loss_U, Urban_Loss_U, Rural_LadB_U, Urban_LadB_U] = UAVAirLossHigh(UAV_height, Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W, Distance_U, MonteCarlo);
[Rural_Loss_H, Urban_Loss_H, Dense_Loss_H, Rural_LadB_H, Urban_LadB_H, Dense_LadB_H] = LEOHAPAirLoss(MonteCarlo, Frequency, Ptx, Gr_H, Gt, Total_loss_tmp_W, Distance_H, Elevation_Angles_steps);
[Rural_Loss_S, Urban_Loss_S, Dense_Loss_S, Rural_LadB_S, Urban_LadB_S, Dense_LadB_S] = LEOHAPAirLoss(MonteCarlo, Frequency, Ptx, Gr_S, Gt, Total_loss_tmp_W, Distance_S, Elevation_Angles_steps);

index_list_U = [1 2 3 4 6 8 10 15 21 33 81];
index_list_H = [1 2 3 5 10 14 21 33 81];
index_list_S = [1 2 4 6 8 10 12 14 17 21 26 32 41 56 81];

for k=1:1:length(index_list_U)
    Rural_PSNR_U_FHSS(k)=sum((Rural_Loss_U(index_list_U(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
end

for k=1:1:length(index_list_H)
    Rural_PSNR_H_FHSS(k)=sum((Rural_Loss_H(index_list_H(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
end

for k=1:1:length(index_list_S)
    Rural_PSNR_S_FHSS(k)=sum((Rural_Loss_S(index_list_S(k),:)./NoisePower_FHSS)>=D_SNR_FHSS)/MonteCarlo;
end
for N_index = 1:length(Nodes)
for c=1:1:length(index_list_S)
[N_index c]
N=Nodes(N_index);
decoded = 0;
decoded_Capture = 0;
discarded = 0;
H_success = 0;
F_success = 0;
    for m=1:1:MonteCarlo
    
     clear TX_stamp
     clear Tsort
     clear TimeStamp
     clear pack_tx_segments

     First_transmit = rand(1,1)/1000;          % First transmission
            
     mu = (1/(N*pkct_p_h)).*Simulation_T;      % inter arrival time
     Inter_arrivals = exprnd(mu,1,N*pkct_p_h); % inter arrival of the traffic in a hour
     Times = [First_transmit Inter_arrivals];
     
     %Next step: convert inter-arrivals into time stamp 
     TimeStamp = cumsum(Times);                % Time stamp of the traffic in the network
     pack_tx_segments=zeros(length(TimeStamp),fragment_length);
            
     %% Time stamp of the hops (segments)
            
        for pack=1:1:length(TimeStamp)   
            for frag = 1:1:fragment_length 
                if frag == 1
                pack_tx_segments(pack,frag) = TimeStamp (pack);
                elseif frag > 1 && frag <=(Header_N_DR8+1)
                pack_tx_segments(pack,frag) = pack_tx_segments(pack,frag-1) + Header_duration;
                elseif frag == (Header_N_DR8+2)
                pack_tx_segments(pack,frag) = pack_tx_segments(pack,frag-1) + Transceiver_wait;
                else
                pack_tx_segments(pack,frag) = pack_tx_segments(pack,frag-1) + fragment_duration;
                end
            end
        end

    %% Vulnerable time
    Transmit = randi(length(TimeStamp));                % Selecting one random device and single transmission instant
    % Ts: transmission started
    Ts= TimeStamp(Transmit);
    
    % vulnerable time
    Tstart = Ts - ToA_DR8(1);
    Tend = Ts + ToA_DR8(1);

%% Find the number active devices when the desired device was transmitting

    index_start = find(pack_tx_segments(:,1)>=Tstart & pack_tx_segments(:,1)<=Tend); % Select all the transmission in 2T interval, where T is on-air time

    simultaneous = (unique([index_start']));
    target_index = find(simultaneous == Transmit);

    simultaneous(target_index) = [];
 
%% Frequency-time scheduling of target transmission: which fragment is using the specific channel for a specific time?
% 
    target_pattern=zeros(1,size(pack_tx_segments,2));
    target_pattern(1) = randi(OBW_channels,1,1);            %First hop of the desired signal

        for assign=2:1:size(pack_tx_segments,2)
            if(assign==Header_N_DR8+1)               
                target_pattern(assign) = 0;                % Do not assign a channel during T_wait 
            else  
                target_pattern(assign) = randi(OBW_channels,1,1);
                dif_track=abs(target_pattern(assign)-target_pattern(assign-1));
                while dif_track < 8                       % 8 x 488 = 3.9 kHz spacing
                    target_pattern(assign) = randi(OBW_channels,1,1);
                    dif_track=abs(target_pattern(assign)-target_pattern(assign-1));
                end
            end 
        end
    
    %% Collision analysis
    target_collided = zeros(1,size(pack_tx_segments,2));           %Collison counter for Desired signal
    target_discarded = zeros(1,size(pack_tx_segments,2));          %Collison counter for Desired signal for capture effect
    frag_active = zeros(1,length(pack_tx_segments(Transmit,:)));
    clear seg_simultaneous
    seg_trans_simultaneous = pack_tx_segments(simultaneous,:);                % Select all the fragments during the interval 2T
    seg_trans_simultaneous(:,(Header_N_DR8+1))=[];                            %Twait, that's not a transmission
    seg_simultaneous = seg_trans_simultaneous;
    %% Following Equation (4), (5), (6) to find A_{H}, A_{F} and A_{L}
        for seg=1:1:length(pack_tx_segments(Transmit,:))
            
            if(seg~=Header_N_DR8+1)
                clear active_pattern
                clear iscollision

                if(seg~=length(pack_tx_segments(Transmit,:)) && seg<=Header_N_DR8)         % collisions during header transmission
                    
                transmission_sim_header(m,seg) = length(find(seg_simultaneous(:,1:Header_N_DR8)>=(pack_tx_segments(Transmit,seg)-Header_duration) & seg_simultaneous(:,1:Header_N_DR8)<=(pack_tx_segments(Transmit,seg)+Header_duration)));
                transmission_sim_frag(m,seg) = length(find(seg_simultaneous(:,(Header_N_DR8+1):end-1)>=(pack_tx_segments(Transmit,seg)-F_duration) & seg_simultaneous(:,(Header_N_DR8+1):end-1)<=(pack_tx_segments(Transmit,seg)+Header_duration)));
                transmission_sim_last(m,seg) = length(find(seg_simultaneous(:,end)>=(pack_tx_segments(Transmit,seg)-Last_fragment_duration) & seg_simultaneous(:,end)<=(pack_tx_segments(Transmit,seg)+Header_duration)));
                
                transmission_sim(m,seg) = transmission_sim_header(m,seg) + transmission_sim_frag(m,seg) + transmission_sim_last(m,seg);
                
                elseif(seg~=length(pack_tx_segments(Transmit,:)) && seg>Header_N_DR8)    % collisions during payload data fragments transmission
                    
                transmission_sim_header(m,seg) = length(find(seg_simultaneous(:,1:Header_N_DR8)>=(pack_tx_segments(Transmit,seg)-Header_duration) & seg_simultaneous(:,1:Header_N_DR8)<=(pack_tx_segments(Transmit,seg)+F_duration)));
                transmission_sim_frag(m,seg) = length(find(seg_simultaneous(:,(Header_N_DR8+1):end-1)>=(pack_tx_segments(Transmit,seg)-F_duration) & seg_simultaneous(:,(Header_N_DR8+1):end-1)<=(pack_tx_segments(Transmit,seg)+F_duration)));
                transmission_sim_last(m,seg) = length(find(seg_simultaneous(:,end)>=(pack_tx_segments(Transmit,seg)-Last_fragment_duration) & seg_simultaneous(:,end)<=(pack_tx_segments(Transmit,seg)+F_duration)));
                
                transmission_sim(m,seg) = transmission_sim_header(m,seg) + transmission_sim_frag(m,seg) + transmission_sim_last(m,seg);
                
                else                                                                     % collisions during the last fragment transmission
                transmission_sim_header(m,seg) = length(find(seg_simultaneous(:,1:Header_N_DR8)>=(pack_tx_segments(Transmit,seg)-Header_duration) & seg_simultaneous(:,1:Header_N_DR8)<=(pack_tx_segments(Transmit,end))+Last_fragment_duration));
                transmission_sim_frag(m,seg) = length(find(seg_simultaneous(:,(Header_N_DR8+1):end-1)>=(pack_tx_segments(Transmit,seg)-F_duration) & seg_simultaneous(:,(Header_N_DR8+1):end-1)<=(pack_tx_segments(Transmit,end))+Last_fragment_duration));
                transmission_sim_last(m,seg) = length(find(seg_simultaneous(:,end)>=pack_tx_segments(Transmit,seg) & seg_simultaneous(:,end)<=(pack_tx_segments(Transmit,end))+Last_fragment_duration));
                
                transmission_sim(m,seg) = transmission_sim_header(m,seg) + transmission_sim_frag(m,seg) + transmission_sim_last(m,seg);
                                
                end   
                if (~isempty(transmission_sim(m,seg)))

                frag_active(seg) =  (transmission_sim(m,seg));
                % Select the channels for the simultenous fragments
                active_pattern = randi(OBW_channels,(transmission_sim(m,seg)),1)';
                % How many active devices are assigned to the same channel (collision)
                iscollision = find(active_pattern==target_pattern(seg));    % if non zero, that's collision
                    
                    if(~isempty(iscollision))
                      target_collided (seg) = 1;
                            
                      % clear Coordinates                                     
                      % clear rho
                      % clear Theta
                      % 
                      % Coordinates=zeros(length(iscollision),2);   
                      % rho = sqrt(rand(length(iscollision),1).*(sqrt(max(Distance_U)^2 - UAV_height^2)^2));       
                      % Theta = rand(length(iscollision),1)*2*pi;                      
                      % Coordinates(:,1) = cos(Theta).*rho;
                      % Coordinates(:,2) = sin(Theta).*rho;
                      % Location_Nodes_Int = sqrt(Coordinates(:,1).^2 + Coordinates(:,2).^2)';
                      % dPropogation=zeros(1,length(iscollision)); 
                      % 
                      % % Distance from interfering nodes to NTN
                      % for track=1:length(Location_Nodes_Int)
                      %    dPropogation = sqrt(UAV_height^2 + Location_Nodes_Int(track).^2); 
                      %    E_AngPro_U = rad2deg(asin(UAV_height/dPropogation));
                      %    Rural_LadB_U_I(track) = Rural_LadB_U(round((E_AngPro_U)-9));
                      %    % [Rural_Loss_U_I, Urban_Loss_U_I, Rural_LadB_U_I(track), Urban_LadB_U_I(track)] = UAVAirLossHigh(UAV_height, Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W, dPropogation(track));
                      % end
                      % pr_h_g_I = sum((Ptx*Gr_U*Gt)./(10.^(Rural_LadB_U_I./10))./Total_loss_tmp_W);
                      % Rural_LadB_U_D = Rural_LadB_U(index_list_U(c));
                      % pr_h_g_D = Ptx*Gr_U*Gt./(10.^(Rural_LadB_U_D./10))./Total_loss_tmp_W;
                     
                      %HAP
                      % clear Coordinates_H                    
                      % clear rho_H 
                      % clear Theta_H 
                      % Coordinates_H = zeros(length(iscollision),2);   
                      % rho_H = sqrt(rand(length(iscollision),1).*(sqrt(max(Distance_H)^2 - Orbital_height_H^2)^2));       
                      % Theta_H = rand(length(iscollision),1)*2*pi;                      
                      % Coordinates_H(:,1) = cos(Theta_H).*rho_H;
                      % Coordinates_H(:,2) = sin(Theta_H).*rho_H;
                      % Location_Nodes_Int_H = sqrt(Coordinates_H(:,1).^2 + Coordinates_H(:,2).^2)';
                      % 
                      % % Distance from interfering nodes to HAP
                      % for track=1:length(Location_Nodes_Int_H)
                      %    dPropogation_H = sqrt(Orbital_height_H^2 + Location_Nodes_Int_H(track).^2); 
                      %    E_AngPro_H = height2el(Orbital_height_H,1,dPropogation_H);
                      %    Rural_LadB_H_I(track) = Rural_LadB_H(round(E_AngPro_H)-9);
                      % end
                      % pr_h_g_I = sum((Ptx*Gr_H*Gt)./(10.^(Rural_LadB_H_I./10))./Total_loss_tmp_W);
                      % Rural_LadB_H_D = Rural_LadB_H(index_list_H(c));             
                      % pr_h_g_D = Ptx*Gr_H*Gt./(10.^(Rural_LadB_H_D./10))./Total_loss_tmp_W;

                      % %Satellite
                      clear Coordinates_S                    
                      clear rho_S 
                      clear Theta_S 
                      Coordinates_S = zeros(length(iscollision),2);   
                      rho_S = sqrt(rand(length(iscollision),1).*(sqrt(max(Distance_S)^2 - Orbital_height_S^2)^2));       
                      Theta_S = rand(length(iscollision),1)*2*pi;                      
                      Coordinates_S(:,1) = cos(Theta_S).*rho_S;
                      Coordinates_S(:,2) = sin(Theta_S).*rho_S;
                      Location_Nodes_Int_S = sqrt(Coordinates_S(:,1).^2 + Coordinates_S(:,2).^2)';

                      for track=1:length(Location_Nodes_Int_S)
                         dPropogation_S = sqrt(Orbital_height_S^2 + Location_Nodes_Int_S(track).^2); 
                         E_AngPro_S = height2el(Orbital_height_S,1,dPropogation_S);
                         Rural_LadB_S_I(track) = Rural_LadB_S(round(E_AngPro_S)-9);
                      end
                      pr_h_g_I = sum((Ptx*Gr_S*Gt)./(10.^(Rural_LadB_S_I./10))./Total_loss_tmp_W);
                      Rural_LadB_S_D = Rural_LadB_S(index_list_S(c));              
                      pr_h_g_D = Ptx*Gr_S*Gt./(10.^(Rural_LadB_S_D./10))./Total_loss_tmp_W;
                      
                      if  pr_h_g_D  < (pr_h_g_I*gamma_th)
                        target_discarded(seg) = 1;
                      end

                    end
                end
            end
        end


%% Decoding
% first three are header
% the fourth is t_wait
% Rest are fragments of 50 ms

Success_header = Header_N_DR8 - length(nonzeros(target_collided(1:Header_N_DR8)));       % No. of successfully received headers
Threshold = size(pack_tx_segments,2) - round(fragment_PHY_length *(1-Code_Rate))-Header_N_DR8 - length(Transceiver_wait);
Success_fragment = size(target_collided,2) - length(nonzeros(target_collided((Header_N_DR8+2):end)))-Header_N_DR8-1;

                      if(Success_fragment>=Threshold)
                           F_success=F_success+1;
                      end

        if (Success_header>=1)
            H_success=H_success+1;
            if(Success_fragment>=Threshold)
                decoded = 1 + decoded;                
            end
        end
        
Success_header_capture =Header_N_DR8 - length(nonzeros(target_discarded(1:Header_N_DR8)));

        if (Success_header_capture>=1)
            Success_fragment_capture = size(target_discarded,2) - length(nonzeros(target_discarded(Header_N_DR8+2:end)))-Header_N_DR8-1;
            if(Success_fragment_capture>=Threshold)
                decoded_Capture = 1 + decoded_Capture;
            end
    
        end        
        
     end
PS_DR8(N_index, c)=decoded;   %Simulated overall success probability
PS_DR8_Capture(N_index, c)=decoded_Capture; %Simulated success probability with capture effect
end
end
% figure
% h(1)=plot(Distance_U(index_list_U),Rural_PSNR_U_FHSS,'--x','SeriesIndex',1,'LineWidth',2.5);
% hold on
% h(2)=plot(Distance_U(index_list_U),PS_DR8_Capture/MonteCarlo,'--o','SeriesIndex',2,'LineWidth',2.5);
% hold on
% h(3)= plot(Distance_U(index_list_U),Rural_PSNR_U_FHSS.*(PS_DR8_Capture/MonteCarlo),'--d','SeriesIndex',3,'LineWidth',2.5);
% hold on
% grid on
% save("FHSS_U_Res_NN.mat", 'PS_DR8_Capture');
% % importdata("FHSS_U_Res.mat");
% ylabel('Success probability', 'Interpreter', 'Latex');
% xlabel('Distance from node to UAV (m)', 'Interpreter', 'Latex');
% axis([Distance_U(end) Distance_U(1) 0 1]);
% legend('$P_{SNR}$', '$P_{SIR}$','$P_{S}$', 'Interpreter', 'Latex');


% figure
% h(1)=plot(Distance_H(index_list_H)/1e3,Rural_PSNR_H_FHSS,'--x','SeriesIndex',1,'LineWidth',2.5);
% hold on
% h(2)=plot(Distance_H(index_list_H)/1e3,PS_DR8_Capture/MonteCarlo,'--o','SeriesIndex',2,'LineWidth',2.5);
% hold on
% h(3)= plot(Distance_H(index_list_H)/1e3,Rural_PSNR_H_FHSS.*(PS_DR8_Capture/MonteCarlo),'--d','SeriesIndex',3,'LineWidth',2.5);
% hold on
% save("FHSS_H_Res_NN.mat", 'PS_DR8_Capture');
% hold on
% grid on
% ylabel('Success probability', 'Interpreter', 'Latex');
% xlabel('Distance from node to HAP (km)', 'Interpreter', 'Latex');
% axis([Distance_H(end)/1e3 Distance_H(1)/1e3 0 1]);
% legend('$P_{SNR}$', '$P_{SIR}$','$P_{S}$', 'Interpreter', 'Latex');
 
% figure
% h(1)=plot(Distance_S(index_list_S)/1e3,Rural_PSNR_S_FHSS,'--x','SeriesIndex',1,'LineWidth',2.5);
% hold on
% h(2)=plot(Distance_S(index_list_S)/1e3,PS_DR8_Capture/MonteCarlo,'--o','SeriesIndex',2,'LineWidth',2.5);
% hold on
% h(3)= plot(Distance_S(index_list_S)/1e3,Rural_PSNR_S_FHSS.*(PS_DR8_Capture/MonteCarlo),'--d','SeriesIndex',3,'LineWidth',2.5);
% hold on
% grid on
save("FHSS_S_Res_NN.mat", 'PS_DR8_Capture');
% % importdata("FHSS_S_Res.mat");
% ylabel('Success probability', 'Interpreter', 'Latex');
% xlabel('Distance from node to Satellite (km)', 'Interpreter', 'Latex');
% axis([Distance_S(end)/1e3 Distance_S(1)/1e3 0 1]);
% legend('$P_{SNR}$', '$P_{SIR}$','$P_{S}$', 'Interpreter', 'Latex');

toc