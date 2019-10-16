%% simulation set
clearvars;
close all;
freq = 1000; %1kHz
Ts = 1/freq; %Sampling time
Tc = 5; %Chirp period
Kt = 1.8; %transformation ratio from V to Nm 

%% tfest fitting part
Nz = 0;%Number of zeros of the plant
Np = 1;%Number of poles of the plant
fmin=0.1; %minimum freq of chirp signal 
fmax=100; %maximum freq of chirp signal
Td = 0:0.1e-3:5e-3;% DelayTime search range: 0 to 5ms with 0.1ms step



%load('result.data'); 

list = ls('ChirpResults/*.csv'); % Check all the files with the extension .csv
list = string(list);

for Nlist = 1:length(list)
    filedir = strcat('ChirpResults/',list(Nlist));
    
    %% read data and set in/out data
    clear result;
    
    result=load(filedir); % Load file data

    time = result(:,1); %time
    in = result(:,2)*Kt; %Input: Torque 
    out = result(:,3); %Output: Angular speed 

    figure(); plot(time,in,time,out);
    xlabel('time[s]')
    ylabel('in[Nm]/out[rad/s]')
    legend('In','Out')
    grid on;

    %% tfestimate to get bodeplot
    in = detrend(in,0); out = detrend(out,0);
    [PXX,FREQ] = tfestimate(in,out,rectwin(Tc/Ts),0,Tc/Ts,freq);%rectangluar window and no overwrap
    Pfrd = frd(PXX,FREQ,'FrequencyUnit','Hz');

    
    % bodeoptions
    opt = bodeoptions('cstprefs');
    opt.PhaseWrapping = 'on';
    opt.PhaseWrappingBranch = -180;
    %{
    figure(2);
    bode(Pfrd,opt);
    title('Freq responce')
    %}
    
    %% mscoherence
    [COHER,cFREQ] = mscohere(in,out,rectwin(Tc/Ts),0,Tc/Ts,freq);

    figure(); semilogx(cFREQ,COHER);
    xlabel('Frequency[Hz]')
    ylabel('Coherence')
    grid on;
    xlim([0.1 500]);
    

    %%%%%% from here Matlab needs SystemIdentificationToolbox %%%%%%%%%%%%%

    [~,kmin]=min(abs(fmin-Pfrd.freq));%search data point index corresponding to fmin
    [~,kmax]=min(abs(fmax-Pfrd.freq));%search data point index corresponding to fmax

    %tfestimation setup
    init_sys = idtf(ones(1,Nz+1),ones(1,Np+1),'IODelay',0e-3);% Nz-order/Np-order TF with 0ms delay

    % constraints for coefficnets of TF
    init_sys.Structure.Numerator.Minimum = 0;
    init_sys.Structure.Numerator.Maximum = inf;
    init_sys.Structure.Denominator.Minimum = 0;
    init_sys.Structure.Denominator.Maximum = inf;
    
    %Coherence shaping
    COHER(COHER<0.9)=0;

    tfopt = tfestOptions('WeightingFilter',COHER(kmin:kmax).*FREQ(kmin:kmax));%Weighting with coherence

    FP = 0;%minimum fit percentage of tfest function

    %Lookup function to find most likely IO delay time
    for DT = 1:length(Td)
        init_sys.IODelay=Td(DT);
        Pest = tfest(fdel(Pfrd,[Pfrd.freq(1:kmin-1);Pfrd.freq(kmax+1:end);]),init_sys,tfopt);
        if FP < Pest.Report.Fit.FitPercent
            FP = Pest.Report.Fit.FitPercent;
            Pest_most = Pest;
        end
    end

    figure();

    bode(Pfrd,opt,Pest_most,opt);
    legend({'Measured data','Estimated'});
    xlim([0.1 500]);
    title(strcat('Freq responce - ',num2str(Pest_most.IODelay*1e3),'ms IODeelay'))

    disp(filedir);
    disp(strcat('J = ',num2str(1/Pest_most.Numerator),'[kgm^2], D = ', num2str(Pest_most.Denominator(2)/Pest_most.Numerator),'[kgm^2s]'))
    
end
