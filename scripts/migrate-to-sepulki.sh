#!/bin/bash

# Sepulki Migration Script
# Migrates from Artifex to the new Sepulki brand and architecture

echo "🔥 Starting Sepulki Migration..."

# Check if git is clean
if [[ -n $(git status --porcelain) ]]; then
    echo "❌ Git working directory is not clean. Please commit or stash changes first."
    exit 1
fi

# Create new branch for migration
git checkout -b migration/sepulki-rebrand

echo "📁 Setting up new repository structure..."

# Create new service directories
mkdir -p services/{foundry-pipeline,anvil-sim,vault-registry,bellows-telemetry,tongs-policy,choreo-dispatch}
mkdir -p packages/{sepulki-sdk}
mkdir -p infrastructure/{kubernetes,terraform}
mkdir -p docs/{api,guides}

# Migrate existing web app to forge-ui
echo "🔄 Migrating web app to forge-ui..."
if [ -d "apps/web" ]; then
    mv apps/web apps/forge-ui
    echo "✅ Renamed apps/web to apps/forge-ui"
fi

# Update package.json names and references
echo "📝 Updating package configurations..."

# Update root package.json
if [ -f "package.json" ]; then
    sed -i.bak 's/"artifex"/"sepulki"/g' package.json
    sed -i.bak 's/@artifex\/web/@sepulki\/forge-ui/g' package.json
    rm package.json.bak
fi

# Update forge-ui package.json
if [ -f "apps/forge-ui/package.json" ]; then
    sed -i.bak 's/"@artifex\/web"/"@sepulki\/forge-ui"/g' apps/forge-ui/package.json
    rm apps/forge-ui/package.json.bak
fi

echo "🎨 Creating brand-aligned service scaffolds..."

# Create basic service structures
for service in foundry-pipeline anvil-sim vault-registry bellows-telemetry tongs-policy choreo-dispatch; do
    if [ ! -d "services/$service" ]; then
        mkdir -p "services/$service/src"
        echo "📦 Created services/$service"
    fi
done

echo "📚 Setting up shared packages..."

# Create shared packages structure
for package in sepulki-sdk; do
    if [ ! -d "packages/$package" ]; then
        mkdir -p "packages/$package/src"
        echo "📦 Created packages/$package"
    fi
done

echo "✅ Migration structure complete!"
echo ""
echo "Next steps:"
echo "1. Review the created directory structure"
echo "2. Run the service scaffolding script"
echo "3. Update git remote to point to new repository"
echo "4. Commit and push the migration branch"
echo ""
echo "To update git remote:"
echo "  git remote set-url origin https://github.com/CatsMeow492/Sepulki.git"