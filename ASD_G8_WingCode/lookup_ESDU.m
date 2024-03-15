function Ks = lookup_ESDU(data,a,b,R,t)

if a>=b
    a_b = a/b;
    b_Rt = b/sqrt(R*t);
    Ks = interp2(data.a_b_BIGa,data.b_Rt_BIGa,data.Ks_BIGa,a_b,b_Rt,'linear');
else
    b_a = b/a;
    a_Rt = a/sqrt(R*t);
    Ks = interp2(data.b_a_BIGb,data.a_Rt_BIGb,data.Ks_BIGb,b_a,a_Rt,'linear');
end

end