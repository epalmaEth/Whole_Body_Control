 time = [1:steps] * TIME_STEP / 1000 ;
 m = 3;  % rows
 n = 3;  % columns
 if 0	
    if steps * TIME_STEP / 1000 < 5 
        figure(1);    sgtitle('CoM');   
        
        	% 	First Column
        subplot(m,n,0*n+1); hold on; grid on;
        plot(time,p_x_collection(),'b');  
        plot(time,p_x_collection(),'bo');
        plot(time(2:end), p_x_ref_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('px and px ref');

        subplot(m,n,1*n+1); hold on; grid on;
        plot(time,p_y_collection(),'b');  
        plot(time,p_y_collection(),'bo');
        plot(time(2:end), p_y_ref_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('py and py ref');

        observer_error_x_collections
        subplot(m,n,2*n+1); hold on; grid on;
        plot(time,observer_error_x_collections(),'b');  
        plot(time,observer_error_x_collections(),'bo');

        plot(time, observer_error_y_collections(), 'g');
        plot(time, observer_error_y_collections(), 'go');
        xlabel('time [s]');   ylabel('py and py ref');
		% 	Second Column
        subplot(m,n,0*n +2); hold on; grid on;
        plot(time,x_x1_collection(),'b');  
        plot(time,x_x1_collection(),'bo');
        plot(time,x_x2_collection(),'g');  
        plot(time,x_x2_collection(),'go');
        plot(time,x_x3_collection(),'c');  
        plot(time,x_x3_collection(),'co');
        plot(time(2:end), desired_x_x_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('X position [m]');

        subplot(m,n,1*n +2); hold on; grid on;
        plot(time,x_y1_collection(),'b');  
        plot(time,x_y1_collection(),'bo');
        plot(time,x_y2_collection(),'g');  
        plot(time,x_y2_collection(),'go');
        plot(time,x_y3_collection(),'c');  
        plot(time,x_y3_collection(),'co');
	plot(time(2:end), desired_x_y_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('Y position [m]');
        
        
        	% 	Third Column
        subplot(m,n,0*n +3); hold on; grid on;
        plot(time,com_x_collection(),'b');  
        plot(time,com_x_collection(),'bo');
        plot(time(2:end), desired_com_x_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('X position [m]');

        subplot(m,n,1*n +3); hold on; grid on;
        plot(time,com_y_collection(),'b');  
        plot(time,com_y_collection(),'bo');
        plot(time(2:end), desired_com_y_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('Y position [m]');
        
        subplot(m,n,2*n +3); hold on; grid on;
        plot(time,com_z_collection(),'b');  
        plot(time,com_z_collection(),'bo');
        plot(time(2:end), desired_com_z_collection(1:end-1), 'ro');
        xlabel('time [s]');   ylabel('Z position [m]');
        
    end

% 
%     figure(2); 
%     sgtitle('Kick leg velocity collection');
%     
%     subplot(1,3,1); xlabel('time [s]'); ylabel('X position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_x(),'b');
%     plot(time,left_foot_velocity_collection_x(),'bo');
% 
%     subplot(1,3,2); xlabel('time [s]'); ylabel('Z position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_z(),'b');
%     plot(time,left_foot_velocity_collection_z(),'bo');
%     
%     subplot(1,3,3); xlabel('time [s]'); ylabel('Y position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_y(),'b');
%     plot(time,left_foot_velocity_collection_y(),'bo');
% 
%     
%     figure(3); 
%     sgtitle('Kick leg angular velocity collection');
%     
%     subplot(1,3,1); xlabel('time [s]'); ylabel('X position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_rotx(),'b');
%     plot(time,left_foot_velocity_collection_rotx(),'bo');
% 
%     subplot(1,3,2); xlabel('time [s]'); ylabel('Z position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_rotz(),'b');
%     plot(time,left_foot_velocity_collection_rotz(),'bo');
%     
%     subplot(1,3,3); xlabel('time [s]'); ylabel('Y position [m]'); hold on; grid on;
%     plot(time,left_foot_velocity_collection_roty(),'b');
%     plot(time,left_foot_velocity_collection_roty(),'bo');

%     if steps * TIME_STEP / 1000 < 3
%         figure(1); hold on; grid on;
%         plot(time, AkneePitch);
%     end

%     if steps * TIME_STEP / 1000 < 3
%         figure(1);
%         
%         subplot(1,3,1); xlabel('time [s]'); ylabel('X position [m]'); hold on; grid on;
%         plot(time,left_foot_velocity_collection_rotx(),'b');
%         plot(time,left_foot_velocity_collection_rotx(),'bo');
% 
%         subplot(1,3,2); xlabel('time [s]'); ylabel('Z position [m]'); hold on; grid on;
%         plot(time,left_foot_velocity_collection_rotz(),'b');
%         plot(time,left_foot_velocity_collection_rotz(),'bo');
% 
%         subplot(1,3,3); xlabel('time [s]'); ylabel('Y position [m]'); hold on; grid on;
%         plot(time,left_foot_velocity_collection_roty(),'b');
%         plot(time,left_foot_velocity_collection_roty(),'bo');
%     end
    
    
    % if your code plots some graphics, it needs to flushed like this:
end
drawnow;
