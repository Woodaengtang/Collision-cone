function err = errFcnSpheroid(velInfo, scale, R_safe, rm)
    cnvntn = simplicationSpheroid(velInfo, rm.rm1, rm.rm2);
    a = R_safe * scale(1);
    err = cnvntn.A1^2 * (1 + cnvntn.tau * cnvntn.V1^2) + cnvntn.A2^2 * (1 + cnvntn.tau * cnvntn.V2^2) + 2 * cnvntn.A1 * cnvntn.A2 * sqrt(1 + cnvntn.tau*(cnvntn.V1^2 + cnvntn.V2^2) + cnvntn.tau^2 * cnvntn.V1^2 * cnvntn.V2^2) - 4*a^2;
end

