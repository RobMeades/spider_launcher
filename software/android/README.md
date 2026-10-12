# Introduction
This folder contains the Android application that controls the spider launcher via Bluetooth.

# How to Use
If you want to build and/or modify this Android application yourself:

- you will need:
  - [Android Studio](https://developer.android.com/studio) (Latest version recommended)
  - [JDK 17](https://www.oracle.com/java/technologies/downloads/) or higher
  - Android SDK 34+
- clone this repository,
- open in Android Studio:
  - `File` -> `Open` and point it to this directory,
  - Wait for the Gradle Sync to finish (this will automatically download the required dependencies),
- build:
  - `Build` -> `Make Project`,
- connect your device in Developer Mode or start an emulator and hit the Run button.

# Downloads
If you would like to just run the pre-built application on an Android phone you can find the latest released `.apk` file in the [Releases](https://github.com/RobMeades/spider_launcher/releases) section of this repository.  Android will show a `Play Protect` warning, which is normal for an independent, open-source project: to install, tap `More Details` and then `Install Anyway`.

Make sure you have Bluetooth enabled on the Android phone and launch the application.  Give the application the permissions it asks for (required to access Bluetooth) and, with the spider launcher powered-up, use the application to scan for and connect to `spider_launcher`.  If the Android phone fails to find `spider_launcher`, try disabling and then re-enabling Bluetooth on the phone and trying again (Android Bluetooth is a mess).
