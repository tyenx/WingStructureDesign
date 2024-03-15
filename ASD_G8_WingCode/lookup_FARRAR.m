function F = lookup_FARRAR(data,As,b,t,ts)

As_bt = As/b/t;
ts_t = ts/t;

F = interp2(data.ts_t,data.As_bt,data.F,ts_t,As_bt,'linear');

end