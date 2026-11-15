# Run only after Cortex-Debug has halted the target without resetting it.
# Output is relative to the launch configuration's cwd (the repository root).

set $sv_up = &_SEGGER_RTT.aUp[1]
set $sv_buf = $sv_up->pBuffer
set $sv_size = $sv_up->SizeOfBuffer
set $sv_rd = $sv_up->RdOff
set $sv_wr = $sv_up->WrOff

printf "SystemView RTT buffer: base=%p size=%u rd=%u wr=%u\n", $sv_buf, $sv_size, $sv_rd, $sv_wr

if $sv_rd == $sv_wr
    printf "SystemView RTT buffer is empty; no file written.\n"
else
    if $sv_rd < $sv_wr
        dump binary memory build/Debug/systemview-postmortem.bin ($sv_buf+$sv_rd) ($sv_buf+$sv_wr)
    else
        dump binary memory build/Debug/systemview-postmortem.bin ($sv_buf+$sv_rd) ($sv_buf+$sv_size)
        if $sv_wr > 0
            append binary memory build/Debug/systemview-postmortem.bin $sv_buf ($sv_buf+$sv_wr)
        end
    end
    printf "Wrote build/Debug/systemview-postmortem.bin\n"
end
