function [ ode_func, tau_func ] = CreateODE(desired_func,control_func,response_func)
%CreateODE Returns derivative of state x at time t, given functions for
%desired state, controller output, and expected system response
%   desired_func: desired = desired_func(t)
%   control_func: tau = control_func(desired,actual)
%   response_func: diff(actual,t) = response_func(actual,tau)
%
%   actual = [q,d_q]
%   desired = [q_d,d_q_d,dd_q_d]
%   tau = column vector of joint torques/forces. same size as q
    
    save_tau=[];
    save_time=[];
    
    ode_func = @ode;
    tau_func = @getTau;
    
    function d_x = ode(t,x)
        sz=size(x);
        desired=desired_func(t);
        actual=reshape(x,[],2);
        
        tau=control_func(desired, actual);
        
        save_tau=[save_tau;tau.'];
        save_time=[save_time;t];
        
        d_x=response_func(actual,tau);
        
        d_x=reshape(d_x,sz);
    end

    function [torques,times] = getTau()
        torques=save_tau;
        times=save_time;
    end
end

