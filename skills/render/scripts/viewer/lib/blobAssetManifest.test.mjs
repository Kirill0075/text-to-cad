import assert from "node:assert/strict";
import test from "node:test";

import {
  applyBlobAssetManifest,
  blobAssetLookupKey,
  normalizeBlobAssetManifest,
} from "./blobAssetManifest.mjs";

test("blobAssetLookupKey normalizes local asset URLs for manifest matching", () => {
  assert.equal(blobAssetLookupKey("/models/sample/.part.step.glb?v=abc"), "/models/sample/.part.step.glb");
  assert.equal(
    blobAssetLookupKey("https://assets.example.test/models/sample%20part.glb?download=1"),
    "/models/sample%20part.glb"
  );
});

test("applyBlobAssetManifest rewrites catalog asset URLs without changing hashes", () => {
  const manifest = normalizeBlobAssetManifest({
    assets: {
      "/models/sample/.part.step.glb": {
        url: "https://assets.example.test/models/sample/.part.step.glb",
      },
    },
  });

  const catalog = applyBlobAssetManifest({
    schemaVersion: 3,
    root: { dir: "models", name: "models", path: "models" },
    entries: [
      {
        file: "sample/part.step",
        kind: "part",
        assets: {
          glb: {
            url: "/models/sample/.part.step.glb?v=old",
            hash: "new-version",
          },
          stepModule: {
            url: "/models/sample/.part.step.js?v=old",
            hash: "module-version",
          },
        },
      },
    ],
  }, manifest);

  assert.equal(
    catalog.entries[0].assets.glb.url,
    "https://assets.example.test/models/sample/.part.step.glb?v=new-version"
  );
  assert.equal(catalog.entries[0].assets.glb.hash, "new-version");
  assert.equal(catalog.entries[0].assets.glb.storage, "vercel-blob");
  assert.equal(catalog.entries[0].assets.stepModule.url, "/models/sample/.part.step.js?v=old");
});
