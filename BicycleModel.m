classdef BicycleModel
        % BicycleModel drive kinematic model
        % Calculates forward and inverse kinematics for a bicycle model
        % Author and Copyright Daniel Barandica 2021

    properties
        % Wheel radius in meters [m]
        WheelRadius  = 1.0;
        
        % Distance from mid of rear wheel to mid of front wheel in meters [m]
        WheelBase  = 1.0;
    end
    
    methods
        function obj = BicycleModel(wheelRadius, WheelBase)
            %BicycleModel Class Constructor
            % Inputs: 
            %   wheelRadius: Wheel radius [m]
            %   WheelBase: Distance from mid of rear wheel to mid of front wheel in meters [m]
            
            % Assign parameters
            obj.WheelRadius = wheelRadius;
            obj.WheelBase = WheelBase;
        end
        
        function [r_xi] = calcForwardKinematics(obj, wr, delta) 
            %CALCFORWARDKINEMATICS Calculates forward kinematics
            % Inputs:
            %    wr: rear wheel speed [rad/s]
            %    delta: angle of direction [rad]
            % Outputs:
            %    r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame                        
            %       vx: robot speed in robot frame [m/s]
            %       wz: robot angular vel in robot frame [rad/s]            
            
            vx = obj.WheelRadius * wr ;
            wz = (obj.WheelRadius *wr * tan(delta))/ obj.WheelBase;
            
            r_xi = [vx;0;wz];
        end
        
        function [wr, delta] = calcInverseKinematics(obj, r_xi)
            %CALCINVERSEKINEMATICS Calculates forward kinematics
            % Inputs:
            %    r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame             
            %       vx: robot linear speed in robot frame  [m/s]
            %       wz: robot angular speed in robot frame  [rad/s]          
            % Outputs:
            %       wr: rear wheel speed [rad/s]
            %       delta: angle of direction [rad]
     
            vx = r_xi(1);
            wz = r_xi(3);

            wr = r_xi(1)/ obj.WheelRadius;
            delta= atan2((wz* obj.WheelBase),vx);            
        end
        
    end   
end        
        