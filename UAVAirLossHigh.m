function [Rural_Loss_U, Urban_Loss_U, UAV_LaRural_dB, UAV_LaUrban_dB] = UAVAirLossHigh(UAV_height,Frequency, Ptx, Gr_U, Gt, Total_loss_tmp_W, Distance, MonteCarlo)
rng('default');
%%%%3GPP TR 38.901: "Study on channel model for frequencies from 0.5 to 100GHz", V14.0.0%%%%
%UAVairLossHigh The UAV height Rural>10m Urban>22.5m
% MonteCarlo = 1e5;
h_bs = 1.5;
h_node = UAV_height;
% Elevation_Angles_steps = 1:1:90;
% Distance_U = height2range(h_node-h,1,Elevation_Angles_steps);
Distance_U = Distance;
%LOS Rural
d1_rural = max(1350.8*log10(h_node)-1602,18);
p1_rural = max(15021*log10(h_node)-16053,1000);
%LOS Urban
d1_urban = max(460*log10(h_node)-700,18);
p1_urban = 4300*log10(h_node)-3800;

for i=1:1:length(Distance_U)
    distance = Distance_U(i);
    r2 = sqrt(distance.^2+(h_bs-h_node).^2);

    if (10<h_node)&&(h_node<=40)
        if distance<=d1_rural
            Pro_LOS_Rural = 1;
        else
            Pro_LOS_Rural = d1_rural/distance+exp(-distance/p1_rural)*(1-d1_rural/distance);
        end
    elseif (40<h_node)&&(h_node<=300)
       Pro_LOS_Rural = 1;
    end

    if (22.5<h_node)&&(h_node<=100)
        if distance<=d1_urban
            Pro_LOS_Urban = 1;
        else
            Pro_LOS_Urban = d1_urban/distance+exp(-distance/p1_urban)*(1-d1_urban/distance);
        end
    elseif (100<h_node)&&(h_node<=300)
       Pro_LOS_Urban = 1;
    end

    parfor mCarlo=1:MonteCarlo
        pro_value = rand;
        if pro_value<=Pro_LOS_Rural
            %Rural LOS case
            La_Rural(i,mCarlo) = max(23.9-1.8*log10(h_node),20)*log10(r2)+20*log10(40*pi*Frequency/1e9/3)+normrnd(0,4.2*exp(-0.0046*h_node));
        else
            %Rural NLOS case
            La_Rural_LOS = max(23.9-1.8*log10(h_node),20)*log10(r2)+20*log10(40*pi*Frequency/1e9/3);
            La_Rural_NLOS = -12+(35-5.3*log10(h_node))*log10(r2)+20*log10(40*pi*Frequency/1e9/3);
            La_Rural(i,mCarlo) = max(La_Rural_LOS, La_Rural_NLOS)+normrnd(0,6);
        end
        
        if pro_value<=Pro_LOS_Urban
            %Urban LOS case
            La_Urban(i,mCarlo) = 22*log10(r2)+28+20*log10(Frequency/1e9)+normrnd(0,4.64*exp(-0.0066*h_node));
        else
            %Urban NLOS case
            La_Urban(i,mCarlo) = -17.5+(46-7*log10(h_node))*log10(r2)+20*log10(40*pi*Frequency/1e9/3)+normrnd(0,6);
        end
    end
    UAV_LaRural_dB(i) = mean(La_Rural(i,:));
    UAV_LaUrban_dB(i) = mean(La_Urban(i,:));
    Rural_Loss_U(i,:) = (Ptx*Gr_U*Gt)./(10.^(La_Rural(i,:)./10))./Total_loss_tmp_W;
    Urban_Loss_U(i,:) = (Ptx*Gr_U*Gt)./(10.^(La_Urban(i,:)./10))./Total_loss_tmp_W;
end

