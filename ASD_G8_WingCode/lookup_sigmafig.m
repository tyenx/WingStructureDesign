function sigma_factor = lookup_sigmafig(data,As,b,t,ts)

As_bt = As/b/t;
ts_t = ts/t;

sigma_factor = interp2(data.ts_t,data.As_bt,data.sigma_factor,ts_t,As_bt,'linear');

end