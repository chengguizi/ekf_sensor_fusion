%% Cheng Huimin @ 2019


% rosshutdown

rosinit;

subState = rossubscriber('/ekf_fusion/state_out', @subCallbackState);


function subCallbackState (source,msg)

end