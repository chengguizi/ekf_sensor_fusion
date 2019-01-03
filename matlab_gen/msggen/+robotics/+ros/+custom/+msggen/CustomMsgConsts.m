classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        sensor_fusion_comm_DoubleArrayStamped = 'sensor_fusion_comm/DoubleArrayStamped'
        sensor_fusion_comm_ExtEkf = 'sensor_fusion_comm/ExtEkf'
        sensor_fusion_comm_ExtState = 'sensor_fusion_comm/ExtState'
        ssf_core_ext_imu = 'ssf_core/ext_imu'
        ssf_core_visensor_imu = 'ssf_core/visensor_imu'
        ssf_updates_PositionWithCovarianceStamped = 'ssf_updates/PositionWithCovarianceStamped'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(6, 1);
                msgList{1} = 'sensor_fusion_comm/DoubleArrayStamped';
                msgList{2} = 'sensor_fusion_comm/ExtEkf';
                msgList{3} = 'sensor_fusion_comm/ExtState';
                msgList{4} = 'ssf_core/ext_imu';
                msgList{5} = 'ssf_core/visensor_imu';
                msgList{6} = 'ssf_updates/PositionWithCovarianceStamped';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
