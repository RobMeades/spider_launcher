pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
    }
}
plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        // Try to workaround the Nordic repository not being available
        maven { url = uri("https.sonatype.org/content/repositories/snapshots") }
        maven { url = uri("https://oss.sonatype.org/content/repositories/releases/") }
    }
}

rootProject.name = "Spider Launcher"
include(":app")
