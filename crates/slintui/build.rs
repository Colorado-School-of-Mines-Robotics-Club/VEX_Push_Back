fn main() {
    // Compile the Slint UI file with the appropriate configuration.
    slint_build::compile_with_config(
        "ui/app.slint", // Path to your Slint UI file.
        slint_build::CompilerConfiguration::new()
            // .with_sdf_fonts(true)
            .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer)
            // Optionally, you can specify a style to use for the UI.
            // Check the Slint documentation for more information.
            .with_style("fluent-dark".into()),
    )
    .expect("Slint build failed");
}
