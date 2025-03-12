pub async fn start(ota_data: &[u8]) -> anyhow::Result<()> {
    let mut flash = esp_storage::FlashStorage::new();
    esp_ota_nostd::ota_begin(&mut flash, ota_data, |_| {})
        .await
        .map_err(|e| anyhow::anyhow!("OTA failed: {:?}", e))?;

    log::info!("[OTA] finished update, rebooting to activate new firmware.");
    esp_hal::reset::software_reset();
    unreachable!()
}
