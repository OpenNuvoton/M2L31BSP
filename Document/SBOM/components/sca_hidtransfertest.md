# HID Transfer Test Tool Component Description (for SCA / SBOM)

This document provides component metadata for the **HID Transfer Test Tool** under `Tool/HIDTransferTest`, so it can be used directly in downstream SBOM generation (CycloneDX format).

## 1) Component Identity

- Component name (`name`): `HID Transfer Test Tool`
- Component type (`type`): `application`
- Supplier / author: `Nuvoton Technology Corporation`
- Version: `1.1.0`
- License: `Nuvoton Proprietary`
- Evidence path: `Tool/HIDTransferTest`
- Primary executable: `Tool/HIDTransferTest/Debug/HIDTransferTest.exe`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Tool/HIDTransferTest/README.txt`
  - Shows `Tool Version: 1.1.0`
  - Describes `HIDTransferTest.exe` command-line usage
- `Tool/HIDTransferTest/HIDTransferTest/version.h`
  - Defines `VER_MAJOR 1`, `VER_MINOR 1`, `VER_REVISION 0`, and `VER_BUILD 0`
  - Defines `VER_FILE_DESCRIPTION` as `HID Transfer Test Tool`
  - Defines `VER_ORIGINAL_FILENAME` as `HIDTransferTest.exe`
- `Tool/HIDTransferTest/Debug/HIDTransferTest.exe`
  - PE version resource shows `FileVersion 1.1.0.0`
  - PE version resource shows `ProductVersion 1.1.0.0`
  - PE version resource shows `ProductName HID Transfer Test Tool`
  - SHA-256: `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a`
- `Tool/HIDTransferTest/LICENSE.md`
  - Nuvoton Software License Agreement
  - Copyright notice for Nuvoton Technology Corporation

## 3) License Handling Guidance

`Tool/HIDTransferTest` is a first-party tool directory distributed under the Nuvoton software license in `Tool/HIDTransferTest/LICENSE.md`.

For CycloneDX output, keep the license as a descriptive license name:

- `licenses[0].license.name`: `Nuvoton Proprietary`

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `application`
- `bom-ref`: `pkg:generic/hid-transfer-test-tool@1.1.0?source=vendored&path=Tool/HIDTransferTest`
- `name`: `HID Transfer Test Tool`
- `version`: `1.1.0`
- `scope`: `required`
- `author`: `Nuvoton Technology Corporation`
- `purl`: `pkg:generic/hid-transfer-test-tool@1.1.0`
- `description`: `HID Transfer Test Tool is a command-line utility for HID interrupt and control transfer testing with supported Nuvoton USB HID devices. The tool is included in the corresponding BSP under Tool/HIDTransferTest.`
- `licenses[0].license.name`: `Nuvoton Proprietary`
- `hashes[0].alg`: `SHA-256`
- `hashes[0].content`: `574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a`
- `properties` (recommended custom properties):
  - `bsp:file-path = Tool/HIDTransferTest`
  - `bsp:primary-executable = Tool/HIDTransferTest/Debug/HIDTransferTest.exe`
  - `integration = vendored_source_and_binary`
  - `bsp:component-origin = first-party`
  - `bsp:component-source = Nuvoton Technology Corporation`
  - `bsp:license-file = Tool/HIDTransferTest/LICENSE.md`
  - `bsp:evidence-file = Document/SBOM/components/sca_hidtransfertest.json`
  - `bsp:evidence-path = Tool/HIDTransferTest`
  - `bsp:version-evidence = Tool/HIDTransferTest/README.txt (Tool Version 1.1.0); Tool/HIDTransferTest/HIDTransferTest/version.h; Tool/HIDTransferTest/Debug/HIDTransferTest.exe (PE version resource: FileVersion 1.1.0.0; ProductVersion 1.1.0.0)`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/hid-transfer-test-tool@1.1.0?source=vendored&path=Tool/HIDTransferTest`
- `purl`: `pkg:generic/hid-transfer-test-tool@1.1.0`

## 6) CycloneDX JSON Component Example

```json
{
  "type": "application",
  "bom-ref": "pkg:generic/hid-transfer-test-tool@1.1.0?source=vendored&path=Tool/HIDTransferTest",
  "name": "HID Transfer Test Tool",
  "version": "1.1.0",
  "scope": "required",
  "author": "Nuvoton Technology Corporation",
  "purl": "pkg:generic/hid-transfer-test-tool@1.1.0",
  "description": "HID Transfer Test Tool is a command-line utility for HID interrupt and control transfer testing with supported Nuvoton USB HID devices. The tool is included in the corresponding BSP under Tool/HIDTransferTest.",
  "licenses": [
    {
      "license": {
        "name": "Nuvoton Proprietary"
      }
    }
  ],
  "hashes": [
    {
      "alg": "SHA-256",
      "content": "574f697b673b96f472321fce24cedbd5a7b3d8711067e23c7b065ae6ab074e1a"
    }
  ],
  "properties": [
    { "name": "bsp:file-path", "value": "Tool/HIDTransferTest" },
    { "name": "bsp:primary-executable", "value": "Tool/HIDTransferTest/Debug/HIDTransferTest.exe" },
    { "name": "integration", "value": "vendored_source_and_binary" },
    { "name": "bsp:component-origin", "value": "first-party" },
    { "name": "bsp:component-source", "value": "Nuvoton Technology Corporation" },
    { "name": "bsp:license-file", "value": "Tool/HIDTransferTest/LICENSE.md" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_hidtransfertest.json" },
    { "name": "bsp:evidence-path", "value": "Tool/HIDTransferTest" },
    { "name": "bsp:version-evidence", "value": "Tool/HIDTransferTest/README.txt (Tool Version 1.1.0); Tool/HIDTransferTest/HIDTransferTest/version.h; Tool/HIDTransferTest/Debug/HIDTransferTest.exe (PE version resource: FileVersion 1.1.0.0; ProductVersion 1.1.0.0)" }
  ]
}
```

## 7) Compliance Notes

- Keep `Tool/HIDTransferTest/LICENSE.md` with the distributed tool directory.
- Treat `Tool/HIDTransferTest` as a first-party vendored source-and-binary tool component in SBOM output.
