import 'package:bookly/core/utils/app_router.dart';
import 'package:bookly/core/utils/assets.dart';
import 'package:bookly/core/widgets/custom_loading_indicator.dart';
import 'package:cached_network_image/cached_network_image.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

class CustomBookImage extends StatelessWidget {
  const CustomBookImage({super.key, required this.imageUrl});
  final String imageUrl;
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(
        horizontal: 8,
      ),
      child: GestureDetector(
        onTap: () {
          GoRouter.of(context).push(AppRouter.kBookDetailsView);
        },
        child: ClipRRect(
          borderRadius: BorderRadius.circular(12),
          child: AspectRatio(
              aspectRatio: 1.4 / 2.1,
              child: CachedNetworkImage(
                errorWidget: (context, url, error) => const Icon(Icons.error),
                placeholder: (context, url) => const CustomLoadingIndicator(),
                imageUrl: imageUrl,
                fit: BoxFit.fill,
              )),
        ),
      ),
    );
  }
}
