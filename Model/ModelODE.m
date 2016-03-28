function [ ode_func, tau_func ] = ModelODE(model,sym_x,c_func)
%ModelODE Creates and ODE using a dynamic model
%   model forward_dynamics are used to simulate system response
%   desired states are determined by differentiating and evaluating sym_x w.r.t model.t
%   c_func specifies a control method to be used. If none is give, the
%       model inverse_dynamics will be used

    n=nargin;
    [ode_func,tau_func]=CreateODE(desired,control,@response);
    
    function d=desired
        desired_t(:,1)=sym_x;
        desired_t(:,2)=diff(sym_x,model.t);
        desired_t(:,3)=diff(desired_t(:,2),model.t);
    
        d=matlabFunction(desired_t);
    end

    function c = control
        if n<3 || isempty(c_func)
            c_func = Controller.ComputedTorque(model);
        end
        c=c_func;
    end

    function r = response(actual,tau)
        r(:,1)=actual(:,2);
        r(:,2)=model.forward_dyn(actual(:,1),actual(:,2),tau);
    end
end

