param(
    [string]$Address = "10.111.97.77",
    [int]$Port = 19020,
    [int]$TimeoutSeconds = 60
)

$deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
Write-Host "Waiting for J-Link Wi-Fi at ${Address}:${Port}..."

do {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $connectTask = $client.ConnectAsync($Address, $Port)
        if ($connectTask.Wait(1000) -and $client.Connected) {
            Write-Host "J-Link Wi-Fi is ready."
            exit 0
        }
    }
    catch {
        # The probe may still be booting or associating with Wi-Fi.
    }
    finally {
        $client.Dispose()
    }

    Start-Sleep -Milliseconds 500
} while ([DateTime]::UtcNow -lt $deadline)

Write-Error "J-Link Wi-Fi did not become ready within ${TimeoutSeconds} seconds."
exit 1
