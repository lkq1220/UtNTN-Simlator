function [Rural_Loss_s, Urban_Loss_s, Dense_Loss_s, Rural_LadB_s, Urban_LadB_s, Dense_LadB_s] = LEOHAPAirLoss(MonteCarlo, Frequency, Ptx, Gr_s, Gt, Total_loss_tmp_W, Distance, Elevation_input)
%% Satellite Geometry
Elevation_Angles = 10:10:90;
Elevation_Angles_steps = 10:1:90;
% Orbital_height=550e3;
% Distance = height2range(Orbital_height,1,Elevation_Angles_steps);

Rural_LOS_prob = [78.2, 86.9, 91.9, 92.9, 93.5, 94.0, 94.9, 95.2, 99.8];
Urban_LOS_prob = [24.6, 38.6, 49.3, 61.3, 72.6, 80.5, 91.9, 96.8, 99.2];
Dense_LOS_prob = [28.2, 33.1, 39.8, 46.8, 53.7, 61.2, 73.8, 82.0, 98.1];

Rural_LOS_prob_all_angles=interp1(Elevation_Angles,Rural_LOS_prob ,Elevation_Angles_steps)/100; % For all angles, probability 0-1
Urban_LOS_prob_all_angles=interp1(Elevation_Angles,Urban_LOS_prob ,Elevation_Angles_steps)/100; % For all angles, probability 0-1
Dense_LOS_prob_all_angles=interp1(Elevation_Angles,Dense_LOS_prob ,Elevation_Angles_steps)/100; % For all angles, probability 0-1

% figure(1)
% Lindwidth_value=2;
% plot(Elevation_Angles_steps,Rural_LOS_prob_all_angles,'r-',LineWidth=Lindwidth_value);
% hold on
% plot(Elevation_Angles_steps,Urban_LOS_prob_all_angles,'g-',LineWidth=Lindwidth_value);
% plot(Elevation_Angles_steps,Dense_LOS_prob_all_angles,'b-',LineWidth=Lindwidth_value);
% xlabel('Elevation angle','Interpreter','Latex','FontSize', 12)
% ylabel('LOS probability','Interpreter','Latex','FontSize', 12)
% axis([Elevation_Angles_steps(1) Elevation_Angles_steps(end) 0 1])
% legend('Rural','Urban','Dense Urban','Location','southeast','Interpreter','Latex','FontSize', 12);
% grid on

%% LOS Shadowing for all angles
Rural_LOS_shadow_fading = [1.79, 1.14, 1.14, 0.92, 1.42, 1.56, 0.85, 0.72, 0.72];  %Rural LOS Shadowing
Urban_LOS_shadow_fading = 4*ones(1,length(Rural_LOS_shadow_fading));  %Urban LOS Shadowing
Dense_LOS_shadow_fading = [3.5, 3.4, 2.9, 3.0, 3.1, 2.7, 2.5, 2.3, 1.2];  %Dense Urban LOS Shadowing

Rural_LOS_shadow_all_angles=interp1(Elevation_Angles,Rural_LOS_shadow_fading ,Elevation_Angles_steps); % For all angles
Urban_LOS_shadow_all_angles=interp1(Elevation_Angles,Urban_LOS_shadow_fading ,Elevation_Angles_steps); 
Dense_LOS_shadow_all_angles=interp1(Elevation_Angles,Dense_LOS_shadow_fading ,Elevation_Angles_steps); 
%  zero-mean normal distribution with a standard deviation 
%% https://se.mathworks.com/help/stats/normrnd.html

%% NLOS Shadowing for all angles
Rural_NLOS_shadow_fading = [8.93, 9.08, 8.78, 10.25, 10.56, 10.74, 10.17, 11.52, 11.52];  %Rural NLOS Shadowing 80-11.52 90-11.52
Urban_NLOS_shadow_fading = 6*ones(1,length(Rural_NLOS_shadow_fading)); %Urban NLOS Shadowing
Dense_NLOS_shadow_fading = [15.5, 13.9, 12.4, 11.7, 10.6, 10.5, 10.1, 9.2, 9.2]; %Dense Urban NLOS Shadowing

Rural_NLOS_shadow_all_angles=interp1(Elevation_Angles,Rural_NLOS_shadow_fading ,Elevation_Angles_steps); % For all angles
Urban_NLOS_shadow_all_angles=interp1(Elevation_Angles,Urban_NLOS_shadow_fading ,Elevation_Angles_steps); % For all angles
Dense_NLOS_shadow_all_angles=interp1(Elevation_Angles,Dense_NLOS_shadow_fading ,Elevation_Angles_steps); % For all angles

%% Clutter losses for NLOS
Rural_NLOS_clutter_loss = [19.52, 18.17, 18.42, 18.28, 18.63, 17.68, 16.50, 16.30, 16.30];
%The clutter losses for Urban and Dense Urban are the same.
Urban_NLOS_clutter_loss = [34.3, 30.9, 29.0, 27.7, 26.8, 26.2, 25.8, 25.5, 25.5];
Dense_NLOS_clutter_loss = [34.3, 30.9, 29.0, 27.7, 26.8, 26.2, 25.8, 25.5, 25.5]; 

Rural_NLOS_clutter_loss_all_angles=interp1(Elevation_Angles,Rural_NLOS_clutter_loss,Elevation_Angles_steps); % For all angles
Urban_NLOS_clutter_loss_all_angles=interp1(Elevation_Angles,Urban_NLOS_clutter_loss,Elevation_Angles_steps); % For all angles
Dense_NLOS_clutter_loss_all_angles=interp1(Elevation_Angles,Dense_NLOS_clutter_loss,Elevation_Angles_steps); % For all angles

%% Shadowing fading ploting
% figure(2)
% F_LoS(1)=plot(Elevation_Angles_steps,Rural_LOS_shadow_all_angles,'r-',LineWidth=Lindwidth_value);
% hold on
% F_LoS(2)=plot(Elevation_Angles_steps,Urban_LOS_shadow_all_angles,'g-',LineWidth=Lindwidth_value);
% F_LoS(3)=plot(Elevation_Angles_steps,Dense_LOS_shadow_all_angles,'b-',LineWidth=Lindwidth_value);
% 
% F_NLoS(1)=plot(Elevation_Angles_steps,Rural_NLOS_shadow_all_angles,'r--',LineWidth=Lindwidth_value);
% hold on
% F_NLoS(2)=plot(Elevation_Angles_steps,Urban_NLOS_shadow_all_angles,'g--',LineWidth=Lindwidth_value);
% F_NLoS(3)=plot(Elevation_Angles_steps,Dense_NLOS_shadow_all_angles,'b--',LineWidth=Lindwidth_value);
% xlabel('Elevation angle','Interpreter','Latex','FontSize', 12)
% ylabel('Shadowing standard deviation (dB)','Interpreter','Latex','FontSize', 12)
% grid on
% hold off
% L_LoS=legend(F_LoS(1:3),{'Rural','Urban','Dense Urban'},'Location','northeast','NumColumns',1, 'Interpreter', 'Latex','FontSize',12);
% title(L_LoS,'LoS')
% L_LoS_ax=axes('Position',get(gca,'Position'),'Visible','Off');
% hold on
% 
% L_NLoS=legend(L_LoS_ax,F_NLoS(1:3),{'Rural','Urban','Dense Urban'},'Location','northeast','NumColumns',1, 'Interpreter', 'Latex','FontSize',12);
% title(L_NLoS,'NLoS')
% L_NLoS_ax=axes('Position',get(gca,'Position'),'Visible','Off');


%% 3GPP path loss model for rural areas
% FSPL = 32.45 + 20*log10(carrier frequency) + 20*log10(distance) in GHz

%% path loss calculations featuring LOS and NLOS probabilities
for index=1:1:length(Elevation_input)
    LOS=0;
    NLOS=0;
    i = Elevation_input(index)-9;
    parfor mCarlo=1:MonteCarlo
        Prob=rand;
        if(Prob<=Rural_LOS_prob_all_angles(i))
            Rural_FSPL(index, mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Rural_LOS_shadow_all_angles(i));
            LOS = LOS + 1;
        else
            Rural_FSPL(index, mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Rural_NLOS_shadow_all_angles(i))+Rural_NLOS_clutter_loss_all_angles(i);
            NLOS = NLOS + 1;
        end

        if(Prob<=Urban_LOS_prob_all_angles(i))
            Urban_FSPL(index, mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Urban_LOS_shadow_all_angles(i));
            %LOS = LOS + 1;
        else
            Urban_FSPL(index, mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Urban_NLOS_shadow_all_angles(i))+Urban_NLOS_clutter_loss_all_angles(i);
            %NLOS = NLOS + 1;
        end

        if(Prob<=Dense_LOS_prob_all_angles(i))
            Dense_FSPL(index, mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Dense_LOS_shadow_all_angles(i));
            %LOS = LOS + 1;
        else
            Dense_FSPL(index ,mCarlo) = -147.55 + 20*log10(Frequency)+20*log10(Distance(index))+normrnd(0,Dense_NLOS_shadow_all_angles(i))+Dense_NLOS_clutter_loss_all_angles(i);
            %NLOS = NLOS + 1;
        end
    end
    % Rural_RSSI(i,:) = (Ptx*Gr*Gt.*h1)./(10.^(Rural_FSPL(i,:)./10))./Total_loss_tmp_W;
    % Urban_RSSI(i,:) = (Ptx*Gr*Gt.*h1)./(10.^(Urban_FSPL(i,:)./10))./Total_loss_tmp_W;
    % Dense_RSSI(i,:) = (Ptx*Gr*Gt.*h1)./(10.^(Dense_FSPL(i,:)./10))./Total_loss_tmp_W;
    Rural_Loss_s(index,:) = (Ptx*Gr_s*Gt)./(10.^(Rural_FSPL(index,:)./10))./Total_loss_tmp_W;
    Urban_Loss_s(index,:) = (Ptx*Gr_s*Gt)./(10.^(Urban_FSPL(index,:)./10))./Total_loss_tmp_W;
    Dense_Loss_s(index,:) = (Ptx*Gr_s*Gt)./(10.^(Dense_FSPL(index,:)./10))./Total_loss_tmp_W;
    Rural_LadB_s(index) = mean(Rural_FSPL(index,:));
    Urban_LadB_s(index) = mean(Urban_FSPL(index,:));
    Dense_LadB_s(index) = mean(Dense_FSPL(index,:));
    % Rural_RSSI_dbm(i,:) = pow2db(Rural_RSSI(i,:)*1000);
    % Urban_RSSI_dbm(i,:) = pow2db(Urban_RSSI(i,:)*1000);
    % Dense_RSSI_dbm(i,:) = pow2db(Dense_RSSI(i,:)*1000);
end